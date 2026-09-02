#include "orthogonal_controller/orthogonal_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace orthogonal_controller
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

void OrthogonalController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  // RCLCPP_INFO(node_->get_logger(), "OrthogonalController configured");
}

void OrthogonalController::cleanup()
{
  // RCLCPP_INFO(node_->get_logger(), "OrthogonalController cleanup");
}

void OrthogonalController::activate()
{
  // RCLCPP_INFO(node_->get_logger(), "OrthogonalController activated");
}

void OrthogonalController::deactivate()
{
  // RCLCPP_INFO(node_->get_logger(), "OrthogonalController deactivated");
}

void OrthogonalController::setPlan(const nav_msgs::msg::Path & path)
{
  // --------------------------------------------------
  // 같은 Goal에 대한 replanning인지, 진짜 새 Goal인지 구분
  // --------------------------------------------------
  bool is_new_goal = true;

  if (!global_plan_.poses.empty() && !path.poses.empty()) {
    const auto & old_goal = global_plan_.poses.back();
    const auto & new_goal = path.poses.back();

    const double dx =
      new_goal.pose.position.x - old_goal.pose.position.x;

    const double dy =
      new_goal.pose.position.y - old_goal.pose.position.y;

    const double old_yaw =
      tf2::getYaw(old_goal.pose.orientation);

    const double new_yaw =
      tf2::getYaw(new_goal.pose.orientation);

    const double dyaw =
      normalizeAngle(new_yaw - old_yaw);

    constexpr double GOAL_POSITION_EPS = 0.001;  // 1 mm
    constexpr double GOAL_YAW_EPS = 0.001;       // 약 0.057 deg

    const bool same_frame =
      global_plan_.header.frame_id == path.header.frame_id;

    const bool same_position =
      std::hypot(dx, dy) <= GOAL_POSITION_EPS;

    const bool same_yaw =
      std::abs(dyaw) <= GOAL_YAW_EPS;

    is_new_goal =
      !(same_frame && same_position && same_yaw);
  }

  // Path 자체는 항상 최신 것으로 교체
  global_plan_ = path;

  // 진짜 새 Goal일 때만 상태 초기화
  if (is_new_goal) {
    rotating_ = false;
    last_corner_index_ =
      std::numeric_limits<std::size_t>::max();

    first_cycle_for_goal_ = true;
    started_inside_final_zone_ = false;
    final_yaw_aligned_ = false;
  }
}

geometry_msgs::msg::TwistStamped
OrthogonalController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = 0.0;
  cmd.twist.angular.x = 0.0;
  cmd.twist.angular.y = 0.0;
  cmd.twist.angular.z = 0.0;

  // --------------------------------------------------
  // 1. Path 존재 확인
  // --------------------------------------------------
  if (global_plan_.poses.empty()) {
    return cmd;
  }

  // --------------------------------------------------
  // 2. 현재 로봇 Pose를 Path 좌표계로 변환
  // --------------------------------------------------
  geometry_msgs::msg::PoseStamped robot_pose;

  try {
    tf_->transform(pose, robot_pose, global_plan_.header.frame_id);
  } catch (const tf2::TransformException & ex) {
    return cmd;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // --------------------------------------------------
  // 3. Goal 거리 / 35cm 진입 여부 계산
  // --------------------------------------------------
  const auto & final_goal =
    global_plan_.poses.back();

  const double final_goal_dx =
    final_goal.pose.position.x - robot_x;

  const double final_goal_dy =
    final_goal.pose.position.y - robot_y;

  const double final_goal_distance =
    std::hypot(final_goal_dx, final_goal_dy);

  // Goal 35cm 이내에서는 회전 금지 + linear 이동만
  constexpr double FINAL_LINEAR_ONLY_DISTANCE = 0.35;

  const bool inside_final_zone =
    final_goal_distance <= FINAL_LINEAR_ONLY_DISTANCE;

  // --------------------------------------------------
  // 3-1. 기존 코너 회전 처리
  //      단, Goal 35cm 이내에서는 코너 회전 취소
  // --------------------------------------------------
  if (rotating_) {

    if (inside_final_zone) {

      rotating_ = false;
      cmd.twist.angular.z = 0.0;

    } else {

      const double yaw_error =
        normalizeAngle(target_yaw_ - robot_yaw);

      if (std::abs(yaw_error) <= 0.08) {

        rotating_ = false;

      } else {

        constexpr double ANGULAR_SPEED = 0.3;

        cmd.twist.angular.z =
          yaw_error > 0.0 ?
          ANGULAR_SPEED :
          -ANGULAR_SPEED;

        return cmd;
      }
    }
  }

  // --------------------------------------------------
  // 4. Goal 최종 접근 처리
  //
  // 기본 규칙
  //   - Goal 50cm ~ 35cm 구간에서 최종 Goal yaw 정렬
  //   - 정렬 완료 후 angular.z = 0
  //   - Goal 35cm 이내에서는 회전 금지
  //   - Goal 35cm 이내에서는 X/Y 중 한 축으로만 이동
  //
  // 예외
  //   - 처음 Goal을 받았을 때부터 35cm 이내라면
  //     처음에 한 번 Goal yaw를 정렬
  // --------------------------------------------------

  constexpr double FINAL_YAW_ALIGN_START_DISTANCE = 0.5;

  // 처음부터 35cm 안에서 시작했는지 기억
  if (first_cycle_for_goal_) {
    started_inside_final_zone_ = inside_final_zone;
    first_cycle_for_goal_ = false;
  }

  // GoalChecker tolerance
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;

  double xy_tolerance = 0.02;
  double yaw_tolerance = 0.08;

  if (goal_checker->getTolerances(
      pose_tolerance,
      vel_tolerance))
  {
    xy_tolerance = pose_tolerance.position.x;

    const double checker_yaw_tolerance =
      std::abs(tf2::getYaw(pose_tolerance.orientation));

    if (checker_yaw_tolerance > 0.0) {
      yaw_tolerance = checker_yaw_tolerance;
    }
  }

  const double goal_yaw =
    tf2::getYaw(final_goal.pose.orientation);

  const double final_yaw_error =
    normalizeAngle(goal_yaw - robot_yaw);

  // --------------------------------------------------
  // 4-2. GoalChecker 기준으로 이미 도착했으면 정지
  // --------------------------------------------------
  const bool goal_reached =
    goal_checker->isGoalReached(
    robot_pose.pose,
    final_goal.pose,
    velocity);

  if (goal_reached) {
    return cmd;
  }

  // --------------------------------------------------
  // 4-3. 최종 Goal yaw 정렬
  //
  // A) 처음부터 35cm 이내에서 시작
  // B) 밖에서 왔다면 50cm ~ 35cm 구간에서 정렬
  // --------------------------------------------------

  const bool align_yaw_from_inside_start =
    started_inside_final_zone_ &&
    !final_yaw_aligned_;

  const bool align_yaw_before_final_zone =
    !started_inside_final_zone_ &&
    !final_yaw_aligned_ &&
    final_goal_distance <= FINAL_YAW_ALIGN_START_DISTANCE &&
    final_goal_distance > FINAL_LINEAR_ONLY_DISTANCE;

  const bool should_align_final_yaw =
    align_yaw_from_inside_start ||
    align_yaw_before_final_zone;

  if (should_align_final_yaw) {

    // 아직 Goal yaw가 tolerance 밖이면 제자리 회전
    if (std::abs(final_yaw_error) > yaw_tolerance) {

      constexpr double FINAL_ANGULAR_SPEED = 0.3;

      cmd.twist.linear.x = 0.0;
      cmd.twist.linear.y = 0.0;

      cmd.twist.angular.z =
        final_yaw_error > 0.0 ?
        FINAL_ANGULAR_SPEED :
        -FINAL_ANGULAR_SPEED;

      return cmd;
    }

    // 한 번 정렬되면 이후에는 다시 회전하지 않도록 기억
    final_yaw_aligned_ = true;
  }

  // --------------------------------------------------
  // 4-4. 최종 접근
  //
  // final_yaw_aligned_ == true
  // 또는 Goal 35cm 이내
  //
  // angular.z = 0
  // X/Y 중 오차가 더 큰 한 축으로만 이동
  // --------------------------------------------------
  if (final_yaw_aligned_ || inside_final_zone) {

    // 최종 접근에서는 회전 금지
    cmd.twist.angular.z = 0.0;

    // --------------------------------------------------
    // XY 위치가 tolerance 안이면 정지
    // --------------------------------------------------
    if (final_goal_distance <= xy_tolerance) {

      cmd.twist.linear.x = 0.0;
      cmd.twist.linear.y = 0.0;

      // XY 도착 후 yaw가 틀어졌다면 마지막으로 방향 보정
      if (std::abs(final_yaw_error) > yaw_tolerance) {

        constexpr double FINAL_ANGULAR_SPEED = 0.3;

        cmd.twist.angular.z =
          final_yaw_error > 0.0 ?
          FINAL_ANGULAR_SPEED :
          -FINAL_ANGULAR_SPEED;

        return cmd;
      }

      cmd.twist.angular.z = 0.0;
      return cmd;
    }

    // --------------------------------------------------
    // Map 기준 Goal 오차를 로봇 기준 좌표로 변환
    // --------------------------------------------------
    const double robot_goal_dx =
      std::cos(robot_yaw) * final_goal_dx +
      std::sin(robot_yaw) * final_goal_dy;

    const double robot_goal_dy =
      -std::sin(robot_yaw) * final_goal_dx +
      std::cos(robot_yaw) * final_goal_dy;

    // --------------------------------------------------
    // 최종 이동 속도
    //
    // X/Y 중 오차가 더 큰 한 축만 사용
    // → 대각선 이동 금지
    // --------------------------------------------------
    constexpr double FINAL_LINEAR_KP = 0.5;
    constexpr double MAX_FINAL_LINEAR_SPEED = 0.08;

    if (std::abs(robot_goal_dx) >= std::abs(robot_goal_dy)) {

      // 앞/뒤로만 이동
      cmd.twist.linear.x = std::clamp(
        FINAL_LINEAR_KP * robot_goal_dx,
        -MAX_FINAL_LINEAR_SPEED,
        MAX_FINAL_LINEAR_SPEED);

      cmd.twist.linear.y = 0.0;

    } else {

      // 좌/우로만 이동
      cmd.twist.linear.x = 0.0;

      cmd.twist.linear.y = std::clamp(
        FINAL_LINEAR_KP * robot_goal_dy,
        -MAX_FINAL_LINEAR_SPEED,
        MAX_FINAL_LINEAR_SPEED);
    }

    // 최종 접근에서는 회전 금지
    cmd.twist.angular.z = 0.0;

    return cmd;
  }

  // --------------------------------------------------
  // 5. 현재 로봇에서 가장 가까운 Path Point 찾기
  // --------------------------------------------------
  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {

    const double dx =
      global_plan_.poses[i].pose.position.x - robot_x;

    const double dy =
      global_plan_.poses[i].pose.position.y - robot_y;

    const double distance =
      std::hypot(dx, dy);

    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  const std::size_t last_index =
    global_plan_.poses.size() - 1;

  // --------------------------------------------------
  // 6. 마지막 Path Point가 가장 가까운 경우
  // --------------------------------------------------
  if (nearest_index >= last_index) {

    const auto & goal =
      global_plan_.poses.back();

    const double goal_dx =
      goal.pose.position.x - robot_x;

    const double goal_dy =
      goal.pose.position.y - robot_y;

    const double goal_distance =
      std::hypot(goal_dx, goal_dy);

    const double robot_dx =
      std::cos(robot_yaw) * goal_dx +
      std::sin(robot_yaw) * goal_dy;

    const double robot_dy =
      -std::sin(robot_yaw) * goal_dx +
      std::cos(robot_yaw) * goal_dy;

    const double LINEAR_SPEED =
      std::clamp(
      goal_distance * 0.8,
      0.02,
      0.08);

    if (std::abs(robot_dx) >= std::abs(robot_dy)) {

      cmd.twist.linear.x =
        robot_dx > 0.0 ?
        LINEAR_SPEED :
        -LINEAR_SPEED;

      cmd.twist.linear.y = 0.0;

    } else {

      cmd.twist.linear.x = 0.0;

      cmd.twist.linear.y =
        robot_dy > 0.0 ?
        LINEAR_SPEED :
        -LINEAR_SPEED;
    }

    cmd.twist.angular.z = 0.0;

    return cmd;
  }

  // --------------------------------------------------
  // 7. 현재 Path Segment
  // --------------------------------------------------
  const auto & p1 =
    global_plan_.poses[nearest_index];

  const auto & p2 =
    global_plan_.poses[nearest_index + 1];

  const double segment_dx =
    p2.pose.position.x -
    p1.pose.position.x;

  const double segment_dy =
    p2.pose.position.y -
    p1.pose.position.y;

  // --------------------------------------------------
  // 8. 코너 확인
  // --------------------------------------------------
  if (nearest_index + 2 < global_plan_.poses.size()) {

    const auto & p3 =
      global_plan_.poses[nearest_index + 2];

    const double dx1 =
      p2.pose.position.x -
      p1.pose.position.x;

    const double dy1 =
      p2.pose.position.y -
      p1.pose.position.y;

    const double dx2 =
      p3.pose.position.x -
      p2.pose.position.x;

    const double dy2 =
      p3.pose.position.y -
      p2.pose.position.y;

    const double cross =
      dx1 * dy2 -
      dy1 * dx2;

    const bool is_corner =
      std::abs(cross) > 0.0001;

    const double corner_dx =
      p2.pose.position.x -
      robot_x;

    const double corner_dy =
      p2.pose.position.y -
      robot_y;

    const double corner_distance =
      std::hypot(
      corner_dx,
      corner_dy);

    if (
      is_corner &&
      corner_distance <= 0.08 &&
      nearest_index + 1 != last_corner_index_)
    {
      target_yaw_ =
        std::atan2(dy2, dx2);

      last_corner_index_ =
        nearest_index + 1;

      rotating_ = true;

      return cmd;
    }
  }

  // --------------------------------------------------
  // 9. 일반 직선 구간 이동
  // --------------------------------------------------
  const double robot_dx =
    std::cos(robot_yaw) * segment_dx +
    std::sin(robot_yaw) * segment_dy;

  const double robot_dy =
    -std::sin(robot_yaw) * segment_dx +
    std::cos(robot_yaw) * segment_dy;

  constexpr double LINEAR_SPEED = 0.13;

  if (std::abs(robot_dx) >= std::abs(robot_dy)) {

    cmd.twist.linear.x =
      robot_dx > 0.0 ?
      LINEAR_SPEED :
      -LINEAR_SPEED;

    cmd.twist.linear.y = 0.0;

  } else {

    cmd.twist.linear.x = 0.0;

    cmd.twist.linear.y =
      robot_dy > 0.0 ?
      LINEAR_SPEED :
      -LINEAR_SPEED;
  }

  // --------------------------------------------------
  // 10. 직선 주행 중 X/Y축 정렬 유지
  // --------------------------------------------------
  double axis_yaw_1;
  double axis_yaw_2;

  if (std::abs(segment_dx) >= std::abs(segment_dy)) {
    axis_yaw_1 = 0.0;
    axis_yaw_2 = M_PI;
  } else {
    axis_yaw_1 = M_PI_2;
    axis_yaw_2 = -M_PI_2;
  }

  const double error_1 =
    normalizeAngle(
    axis_yaw_1 - robot_yaw);

  const double error_2 =
    normalizeAngle(
    axis_yaw_2 - robot_yaw);

  const double yaw_error =
    std::abs(error_1) <= std::abs(error_2) ?
    error_1 :
    error_2;

  constexpr double YAW_DEADBAND = 0.052;
  constexpr double YAW_KP = 1.0;
  constexpr double MAX_ANGULAR_SPEED = 0.3;

  if (std::abs(yaw_error) <= YAW_DEADBAND) {

    cmd.twist.angular.z = 0.0;

  } else {

    cmd.twist.angular.z =
      std::clamp(
      YAW_KP * yaw_error,
      -MAX_ANGULAR_SPEED,
      MAX_ANGULAR_SPEED);
  }

  return cmd;
}

void OrthogonalController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  (void)speed_limit;
  (void)percentage;
}

}  // namespace orthogonal_controller

PLUGINLIB_EXPORT_CLASS(
  orthogonal_controller::OrthogonalController,
  nav2_core::Controller)