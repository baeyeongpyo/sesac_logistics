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

    // RCLCPP_WARN(
      // node_->get_logger(),
      // "[SET PLAN] same_goal=%s goal_delta=%.6f yaw_delta=%.6f",
      // is_new_goal ? "FALSE" : "TRUE",
      // std::hypot(dx, dy),
      // dyaw);
  }

  // Path 자체는 항상 최신 것으로 교체
  global_plan_ = path;

  // --------------------------------------------------
  // 진짜 새 Goal일 때만 상태 초기화
  // 같은 Goal의 replanning이면 회전/yaw 상태를 유지한다.
  // --------------------------------------------------
  if (is_new_goal) {
    rotating_ = false;
    last_corner_index_ =
      std::numeric_limits<std::size_t>::max();

    first_cycle_for_goal_ = true;
    started_inside_final_zone_ = false;
    final_yaw_aligned_ = false;

    // RCLCPP_WARN(
      // node_->get_logger(),
      // "[SET PLAN] NEW GOAL -> controller state reset");

  } else {
    // RCLCPP_WARN(
      // node_->get_logger(),
      // "[SET PLAN] SAME GOAL REPLAN -> controller state preserved "
      // "(rotating=%s yaw_aligned=%s)",
      // rotating_ ? "TRUE" : "FALSE",
      // final_yaw_aligned_ ? "TRUE" : "FALSE");
  }

  // RCLCPP_INFO(
    // node_->get_logger(),
    // "Received global plan with %zu poses",
    // global_plan_.poses.size());

  // DEBUG: Planner가 실제로 넘긴 Path의 마지막 점 확인
  if (!global_plan_.poses.empty()) {
    const auto & path_goal = global_plan_.poses.back();
    const double path_goal_yaw = tf2::getYaw(path_goal.pose.orientation);

    // RCLCPP_WARN(
      // node_->get_logger(),
      // "[DEBUG PATH GOAL] frame=%s x=%.4f y=%.4f yaw=%.4f rad (%.2f deg)",
      // global_plan_.header.frame_id.c_str(),
      // path_goal.pose.position.x,
      // path_goal.pose.position.y,
      // path_goal_yaw,
      // path_goal_yaw * 180.0 / M_PI);
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
    // RCLCPP_WARN(node_->get_logger(), "Global plan is empty");
    return cmd;
  }

  // --------------------------------------------------
  // 2. 현재 로봇 Pose를 Path 좌표계로 변환
  // --------------------------------------------------

  geometry_msgs::msg::PoseStamped robot_pose;

  try {
    tf_->transform(pose, robot_pose, global_plan_.header.frame_id);
  } catch (const tf2::TransformException & ex) {
    // RCLCPP_WARN(node_->get_logger(), "TF transform failed: %s", ex.what());
    return cmd;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // RCLCPP_WARN(
    // node_->get_logger(),
    // "[DEBUG FRAME] input_frame=%s transformed_robot_frame=%s plan_frame=%s "
    // "input=(%.4f, %.4f) transformed_robot=(%.4f, %.4f)",
    // pose.header.frame_id.c_str(),
    // robot_pose.header.frame_id.c_str(),
    // global_plan_.header.frame_id.c_str(),
    // pose.pose.position.x,
    // pose.pose.position.y,
    // robot_x,
    // robot_y);

  // --------------------------------------------------
  // 3. Goal 거리 / 40cm 진입 여부 먼저 계산
  // --------------------------------------------------
  //
  // 코너 회전 처리보다 먼저 Goal 40cm 안인지 알아야
  // 40cm 안에서는 기존 코너 회전을 즉시 막을 수 있다.
  // --------------------------------------------------

  const auto & final_goal =
    global_plan_.poses.back();

  const double final_goal_dx =
    final_goal.pose.position.x - robot_x;

  const double final_goal_dy =
    final_goal.pose.position.y - robot_y;

  const double final_goal_distance =
    std::hypot(final_goal_dx, final_goal_dy);

  constexpr double FINAL_LINEAR_ONLY_DISTANCE = 0.5;

  const bool inside_final_zone =
    final_goal_distance <= FINAL_LINEAR_ONLY_DISTANCE;

  // --------------------------------------------------
  // 3-1. 코너 회전 중이라면 회전부터 완료
  //      단, Goal 40cm 안에서는 코너 회전 금지
  // --------------------------------------------------

  if (rotating_) {

    // Goal 40cm 안으로 들어왔다면 기존 코너 회전을 즉시 취소
    if (inside_final_zone) {

      rotating_ = false;
      cmd.twist.angular.z = 0.0;

      // RCLCPP_INFO(
        // node_->get_logger(),
        // "CANCEL CORNER ROTATION: inside final 40cm zone");

    } else {

      const double yaw_error =
        normalizeAngle(target_yaw_ - robot_yaw);

      // RCLCPP_INFO(
        // node_->get_logger(),
        // "ROTATING: robot_yaw=%.3f target_yaw=%.3f yaw_error=%.3f",
        // robot_yaw,
        // target_yaw_,
        // yaw_error);

      if (std::abs(yaw_error) <= 0.08) {

        rotating_ = false;

        // RCLCPP_INFO(
          // node_->get_logger(),
          // "Rotation finished. robot_yaw=%.3f target_yaw=%.3f",
          // robot_yaw,
          // target_yaw_);

      } else {

        constexpr double ANGULAR_SPEED = 0.3;

        cmd.twist.angular.z =
          yaw_error > 0.0 ?
          ANGULAR_SPEED :
          -ANGULAR_SPEED;

        // RCLCPP_INFO(
          // node_->get_logger(),
          // "ROTATE CMD: angular.z=%.3f",
          // cmd.twist.angular.z);

        return cmd;
      }
    }
  }

  // --------------------------------------------------
  // 4. Goal 최종 접근 처리
  // --------------------------------------------------
  //
  // 기본 규칙
  //   - Goal 60cm ~ 40cm 구간에서 최종 Goal yaw를 미리 정렬한다.
  //   - 최종 yaw 정렬이 끝난 뒤에는 angular.z = 0으로 고정한다.
  //   - Goal 40cm 이내에서는 무조건 회전하지 않고 linear.x / linear.y만 사용한다.
  //
  // 예외
  //   - Goal을 처음 받았을 때부터 이미 40cm 이내였다면,
  //     밖에서 yaw를 맞출 기회가 없으므로 처음에 한 번 Goal yaw를 정렬한다.
  //   - 그 정렬이 끝나면 이후에는 역시 angular.z = 0으로 고정한다.
  // --------------------------------------------------

  constexpr double FINAL_YAW_ALIGN_START_DISTANCE = 0.7;

  // --------------------------------------------------
  // 4-1. 이 Goal을 처음 받았을 때 40cm 안에서 시작했는지 기억
  // --------------------------------------------------

  if (first_cycle_for_goal_) {
    started_inside_final_zone_ = inside_final_zone;
    first_cycle_for_goal_ = false;

    // RCLCPP_INFO(
      // node_->get_logger(),
      // "GOAL START STATE: distance=%.3f started_inside_40cm=%s",
      // final_goal_distance,
      // started_inside_final_zone_ ? "TRUE" : "FALSE");
  }

  // GoalChecker가 사용하는 tolerance를 그대로 가져온다.
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
  // 4-2. 이미 GoalChecker 기준으로 도착했다면 정지
  // --------------------------------------------------

  const bool goal_reached =
    goal_checker->isGoalReached(
    robot_pose.pose,
    final_goal.pose,
    velocity);

  // RCLCPP_INFO(
    // node_->get_logger(),
    // "FINAL STATE: distance=%.3f inside_40cm=%s "
    // "started_inside_40cm=%s yaw_aligned=%s "
    // "yaw_error=%.3f xy_tol=%.3f yaw_tol=%.3f",
    // final_goal_distance,
    // inside_final_zone ? "TRUE" : "FALSE",
    // started_inside_final_zone_ ? "TRUE" : "FALSE",
    // final_yaw_aligned_ ? "TRUE" : "FALSE",
    // final_yaw_error,
    // xy_tolerance,
    // yaw_tolerance);

  if (goal_reached) {
    // RCLCPP_INFO(
      // node_->get_logger(),
      // "Goal reached by GoalChecker");

    return cmd;
  }

  // --------------------------------------------------
  // 4-3. 최종 Goal yaw 정렬이 필요한지 판단
  // --------------------------------------------------
  //
  // 경우 A)
  //   처음부터 40cm 안에서 시작했다면 예외적으로 먼저 yaw 정렬
  //
  // 경우 B)
  //   40cm 밖에서 시작했다면 60cm ~ 40cm 구간에서 미리 yaw 정렬
  // --------------------------------------------------

  //started_inside_final_zone_ == true
  // → 애초에 Goal 40cm 안에서 시작했음

  // final_yaw_aligned_ == false
  // → 아직 Goal yaw를 맞춘 적 없음
  //즉, "처음부터 40cm 안이었고, 아직 최종 방향도 안 맞췄다."
  const bool align_yaw_from_inside_start =
    started_inside_final_zone_ &&
    !final_yaw_aligned_;

  //처음에는 40cm 밖에서 출발했고, 아직 최종 yaw 정렬 안 했고, Goal 거리 <= 60cm , Goal 거리 > 40cm
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

      // RCLCPP_INFO(
        // node_->get_logger(),
        // "FINAL YAW ALIGN: distance=%.3f yaw_error=%.3f angular.z=%.3f",
        // final_goal_distance,
        // final_yaw_error,
        // cmd.twist.angular.z);

      return cmd;
    }

    // yaw가 tolerance 안으로 들어왔으면 정렬 완료를 기억
    // RCLCPP_WARN(
      // node_->get_logger(),
      // "[YAW FLAG SET TRUE] "
      // "distance=%.3f yaw_error=%.3f rad (%.2f deg) "
      // "yaw_tolerance=%.3f robot_yaw=%.3f goal_yaw=%.3f",
      // final_goal_distance,
      // final_yaw_error,
      // final_yaw_error * 180.0 / M_PI,
      // yaw_tolerance,
      // robot_yaw,
      // goal_yaw);

    final_yaw_aligned_ = true;

    // RCLCPP_INFO(
      // node_->get_logger(),
      // "FINAL YAW ALIGNED: distance=%.3f robot_yaw=%.3f goal_yaw=%.3f",
      // final_goal_distance,
      // robot_yaw,
      // goal_yaw);
  }

   // --------------------------------------------------
  // 4-4. 최종 yaw 정렬 완료 후 또는 Goal 40cm 이내
  //      회전 금지 + X/Y 동시 이동
  // --------------------------------------------------
  //
  // final_yaw_aligned_ == true:
  //   최종 Goal yaw를 이미 맞췄으므로 더 이상 회전하지 않는다.
  //
  // inside_final_zone == true:
  //   Goal 40cm 이내에서는 angular.z = 0으로 고정한다.
  //
  // 이후 linear.x / linear.y를 동시에 사용해서
  // Goal 위치까지 대각선 이동할 수 있다.
  // --------------------------------------------------

  if (final_yaw_aligned_ || inside_final_zone) {

    if (final_yaw_aligned_) {
      // RCLCPP_WARN(
        // node_->get_logger(),
        // "[YAW AFTER ALIGNED] "
        // "distance=%.3f yaw_error=%.3f rad (%.2f deg) "
        // "robot_yaw=%.3f goal_yaw=%.3f",
        // final_goal_distance,
        // final_yaw_error,
        // final_yaw_error * 180.0 / M_PI,
        // robot_yaw,
        // goal_yaw);
    }

    // 최종 접근에서는 회전 금지
    cmd.twist.angular.z = 0.0;

    // --------------------------------------------------
    // XY 위치가 tolerance 안이면 정지
    // --------------------------------------------------

    if (final_goal_distance <= xy_tolerance) {

      cmd.twist.linear.x = 0.0;
      cmd.twist.linear.y = 0.0;

      // XY 위치는 도착했지만 yaw가 다시 tolerance 밖으로 틀어졌다면
      // 마지막으로 제자리 회전해서 최종 방향을 다시 맞춘다.
      if (std::abs(final_yaw_error) > yaw_tolerance) {

        constexpr double FINAL_ANGULAR_SPEED = 0.3;

        cmd.twist.angular.z =
          final_yaw_error > 0.0 ?
          FINAL_ANGULAR_SPEED :
          -FINAL_ANGULAR_SPEED;

        // RCLCPP_WARN(
          // node_->get_logger(),
          // "[FINAL YAW RE-ALIGN] "
          // "distance=%.3f yaw_error=%.3f rad (%.2f deg) "
          // "yaw_tol=%.3f angular.z=%.3f",
          // final_goal_distance,
          // final_yaw_error,
          // final_yaw_error * 180.0 / M_PI,
          // yaw_tolerance,
          // cmd.twist.angular.z);

        return cmd;
      }

      // XY와 yaw가 모두 tolerance 안이면 완전히 정지
      cmd.twist.angular.z = 0.0;

      // RCLCPP_INFO(
        // node_->get_logger(),
        // "FINAL POSE reached: distance=%.3f yaw_error=%.3f",
        // final_goal_distance,
        // final_yaw_error);

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
    // X/Y를 동시에 제어
    //
    // 오차가 클수록 빠르게,
    // Goal에 가까워질수록 느리게 이동
    //
    // 최대 속도: 0.08 m/s
    // --------------------------------------------------

    constexpr double FINAL_LINEAR_KP = 0.5;
    constexpr double MAX_FINAL_LINEAR_SPEED = 0.08;

    cmd.twist.linear.x = std::clamp(
      FINAL_LINEAR_KP * robot_goal_dx,
      -MAX_FINAL_LINEAR_SPEED,
      MAX_FINAL_LINEAR_SPEED);

    cmd.twist.linear.y = std::clamp(
      FINAL_LINEAR_KP * robot_goal_dy,
      -MAX_FINAL_LINEAR_SPEED,
      MAX_FINAL_LINEAR_SPEED);

    // 최종 접근에서는 절대 회전하지 않는다.
    cmd.twist.angular.z = 0.0;

    // RCLCPP_INFO(
      // node_->get_logger(),
      // "FINAL XY MOVE: "
      // "distance=%.3f "
      // "robot_dx=%.3f robot_dy=%.3f "
      // "cmd_x=%.3f cmd_y=%.3f angular=%.3f",
      // final_goal_distance,
      // robot_goal_dx,
      // robot_goal_dy,
      // cmd.twist.linear.x,
      // cmd.twist.linear.y,
      // cmd.twist.angular.z);

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

    const double distance = std::hypot(dx, dy);

    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  const std::size_t last_index =
    global_plan_.poses.size() - 1;

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "PATH STATE: nearest_index=%zu last_index=%zu distance=%.3f",
  //   nearest_index,
  //   last_index,
  //   nearest_distance);

  // --------------------------------------------------
  // 6. 마지막 Path Point가 가장 가까운 경우
  // --------------------------------------------------
  //
  // 10cm보다 멀지만 nearest point가 마지막 점인 경우
  // 기존 방식으로 Goal을 향해 이동한다.
  // --------------------------------------------------

  if (nearest_index >= last_index) {

    const auto & goal = global_plan_.poses.back();

    const double goal_dx =
      goal.pose.position.x - robot_x;

    const double goal_dy =
      goal.pose.position.y - robot_y;

    const double goal_distance =
      std::hypot(goal_dx, goal_dy);

    // RCLCPP_INFO(
    //   node_->get_logger(),
    //   "FINAL APPROACH OUTSIDE 10CM: goal_distance=%.3f",
    //   goal_distance);

    const double robot_dx =
      std::cos(robot_yaw) * goal_dx +
      std::sin(robot_yaw) * goal_dy;

    const double robot_dy =
      -std::sin(robot_yaw) * goal_dx +
      std::cos(robot_yaw) * goal_dy;

    const double LINEAR_SPEED = std::clamp(
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

    // RCLCPP_INFO(
    //   node_->get_logger(),
    //   "FINAL MOVE CMD: "
    //   "distance=%.3f speed=%.3f x=%.3f y=%.3f",
    //   goal_distance,
    //   LINEAR_SPEED,
    //   cmd.twist.linear.x,
    //   cmd.twist.linear.y);

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

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "SEGMENT: index=%zu -> %zu dx=%.3f dy=%.3f",
  //   nearest_index,
  //   nearest_index + 1,
  //   segment_dx,
  //   segment_dy);

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

    const bool is_corner = //p2 = corner로 판정
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
      corner_distance <= 0.08 && //로봇이 코너 지점(p2)에서 8cm 이내까지 가까워졌는가?
      nearest_index + 1 != last_corner_index_) //p2
    {
      target_yaw_ =
        std::atan2(dy2, dx2);

      last_corner_index_ =
        nearest_index + 1;

      rotating_ = true; //로봇이 P2의 8cm 안에 들어오면 true

      // RCLCPP_INFO(
        // node_->get_logger(),
        // "Corner detected. Start rotation. "
        // "corner_index=%zu target_yaw=%.3f",
        // nearest_index + 1,
        // target_yaw_);

      return cmd; 
    }
  }

  // --------------------------------------------------
  // 9. 일반 직선 구간 이동. 즉 코너가 아닐 때
  // --------------------------------------------------

  const double robot_dx =
    std::cos(robot_yaw) * segment_dx +
    std::sin(robot_yaw) * segment_dy;

  const double robot_dy =
    -std::sin(robot_yaw) * segment_dx +
    std::cos(robot_yaw) * segment_dy;

  // RCLCPP_INFO(
    // node_->get_logger(),
    // "BODY SEGMENT: robot_dx=%.3f robot_dy=%.3f",
    // robot_dx,
    // robot_dy);

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
    axis_yaw_1 = 0.0;   // +X를 바라봄
    axis_yaw_2 = M_PI;  // -X를 바라봄
  } else {
    axis_yaw_1 = M_PI_2; //+y를 바라봄
    axis_yaw_2 = -M_PI_2; //-y를 바라봄
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

  // RCLCPP_INFO(
    // node_->get_logger(),
    // "STRAIGHT ALIGN: "
    // "robot_yaw=%.3f target_yaw=%.3f "
    // "yaw_error=%.3f angular=%.3f",
    // robot_yaw,
    // normalizeAngle(robot_yaw + yaw_error),
    // yaw_error,
    // cmd.twist.angular.z);

  // RCLCPP_INFO(
    // node_->get_logger(),
    // "MOVE CMD: x=%.3f y=%.3f angular=%.3f",
    // cmd.twist.linear.x,
    // cmd.twist.linear.y,
    // cmd.twist.angular.z);

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
