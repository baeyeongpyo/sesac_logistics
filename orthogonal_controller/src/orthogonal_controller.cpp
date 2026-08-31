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

  RCLCPP_INFO(node_->get_logger(), "OrthogonalController configured");
}

void OrthogonalController::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "OrthogonalController cleanup");
}

void OrthogonalController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "OrthogonalController activated");
}

void OrthogonalController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "OrthogonalController deactivated");
}

void OrthogonalController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;

  RCLCPP_INFO(
    node_->get_logger(),
    "Received global plan with %zu poses",
    global_plan_.poses.size());
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
    RCLCPP_WARN(node_->get_logger(), "Global plan is empty");
    return cmd;
  }

  // --------------------------------------------------
  // 2. 현재 로봇 Pose를 Path 좌표계로 변환
  // --------------------------------------------------

  geometry_msgs::msg::PoseStamped robot_pose;

  try {
    tf_->transform(pose, robot_pose, global_plan_.header.frame_id);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "TF transform failed: %s", ex.what());
    return cmd;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // --------------------------------------------------
  // 3. 코너 회전 중이라면 회전부터 완료
  // --------------------------------------------------

  if (rotating_) {
    const double yaw_error = normalizeAngle(target_yaw_ - robot_yaw);

    RCLCPP_INFO(
      node_->get_logger(),
      "ROTATING: robot_yaw=%.3f target_yaw=%.3f yaw_error=%.3f",
      robot_yaw, target_yaw_, yaw_error);

    if (std::abs(yaw_error) <= 0.08) {
      rotating_ = false;

      RCLCPP_INFO(
        node_->get_logger(),
        "Rotation finished. robot_yaw=%.3f target_yaw=%.3f",
        robot_yaw, target_yaw_);
    } else {
      constexpr double ANGULAR_SPEED = 0.3;

      cmd.twist.angular.z =
        yaw_error > 0.0 ? ANGULAR_SPEED : -ANGULAR_SPEED;

      RCLCPP_INFO(
        node_->get_logger(),
        "ROTATE CMD: angular.z=%.3f",
        cmd.twist.angular.z);

      return cmd;
    }
  }

  // --------------------------------------------------
  // 4. 현재 로봇에서 가장 가까운 Path Point 찾기
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

  const std::size_t last_index = global_plan_.poses.size() - 1;

  RCLCPP_INFO(
    node_->get_logger(),
    "PATH STATE: nearest_index=%zu last_index=%zu distance=%.3f",
    nearest_index, last_index, nearest_distance);

  // --------------------------------------------------
  // 5. 마지막 Path Point가 가장 가까운 경우
  // --------------------------------------------------

  if (nearest_index >= last_index) {
    const auto & goal = global_plan_.poses.back();

    const double goal_dx = goal.pose.position.x - robot_x;
    const double goal_dy = goal.pose.position.y - robot_y;
    const double goal_distance = std::hypot(goal_dx, goal_dy);

    RCLCPP_INFO(
      node_->get_logger(),
      "FINAL APPROACH: goal_distance=%.3f",
      goal_distance);

    // --------------------------------------------------
    // DEBUG: GoalChecker가 실제로 비교하는 값 확인
    // --------------------------------------------------

    const double goal_yaw_debug =
      tf2::getYaw(goal.pose.orientation);

    const double yaw_error_debug =
      normalizeAngle(goal_yaw_debug - robot_yaw);

    geometry_msgs::msg::Pose pose_tolerance_debug;
    geometry_msgs::msg::Twist vel_tolerance_debug;

    double xy_tolerance_debug = -1.0;
    double yaw_tolerance_debug = -1.0;

    const bool got_tolerance =
      goal_checker->getTolerances(
      pose_tolerance_debug,
      vel_tolerance_debug);

    if (got_tolerance) {
      xy_tolerance_debug =
        pose_tolerance_debug.position.x;

      yaw_tolerance_debug =
        tf2::getYaw(pose_tolerance_debug.orientation);
    }

    const bool goal_reached =
      goal_checker->isGoalReached(
      robot_pose.pose,
      goal.pose,
      velocity);

    RCLCPP_WARN(
      node_->get_logger(),
      "GOAL DEBUG: "
      "robot=(%.3f, %.3f) "
      "goal=(%.3f, %.3f) "
      "dist=%.3f "
      "robot_yaw=%.3f "
      "goal_yaw=%.3f "
      "yaw_error=%.3f "
      "xy_tol=%.3f "
      "yaw_tol=%.3f "
      "reached=%s",
      robot_x,
      robot_y,
      goal.pose.position.x,
      goal.pose.position.y,
      goal_distance,
      robot_yaw,
      goal_yaw_debug,
      yaw_error_debug,
      xy_tolerance_debug,
      yaw_tolerance_debug,
      goal_reached ? "TRUE" : "FALSE");

    // Nav2 GoalChecker 기준으로 최종 도착 여부 확인
    if (goal_reached) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Goal reached by Nav2 GoalChecker");

      return cmd;
    }

    // GoalChecker가 사용하는 tolerance 가져오기
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist vel_tolerance;

    if (goal_checker->getTolerances(
        pose_tolerance,
        vel_tolerance))
    {
      const double xy_tolerance =
        pose_tolerance.position.x;

      const double yaw_tolerance =
        tf2::getYaw(pose_tolerance.orientation);

      RCLCPP_INFO(
        node_->get_logger(),
        "GOAL TOLERANCE: xy=%.3f yaw=%.3f",
        xy_tolerance,
        yaw_tolerance);

      // 위치는 도착했는데 yaw만 아직 안 맞음
      if (goal_distance <= xy_tolerance) {
        const double goal_yaw =
          tf2::getYaw(goal.pose.orientation);

        const double yaw_error =
          normalizeAngle(goal_yaw - robot_yaw);

        RCLCPP_INFO(
          node_->get_logger(),
          "FINAL YAW: robot_yaw=%.3f goal_yaw=%.3f yaw_error=%.3f",
          robot_yaw,
          goal_yaw,
          yaw_error);

        constexpr double FINAL_ANGULAR_SPEED = 0.3;

        cmd.twist.angular.z =
          yaw_error > 0.0 ?
          FINAL_ANGULAR_SPEED :
          -FINAL_ANGULAR_SPEED;

        RCLCPP_INFO(
          node_->get_logger(),
          "FINAL ROTATE CMD: angular.z=%.3f",
          cmd.twist.angular.z);

        return cmd;
      }
    }

    // --------------------------------------------------
    // Goal 위치까지 아직 멂 → 가까워질수록 감속
    // --------------------------------------------------

    const double robot_dx =
      std::cos(robot_yaw) * goal_dx +
      std::sin(robot_yaw) * goal_dy;

    const double robot_dy =
      -std::sin(robot_yaw) * goal_dx +
      std::cos(robot_yaw) * goal_dy;

    // Goal에 가까워질수록 속도를 줄임
    // 최소 0.02 m/s, 최대 0.08 m/s
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

    RCLCPP_INFO(
      node_->get_logger(),
      "FINAL MOVE CMD: distance=%.3f speed=%.3f x=%.3f y=%.3f",
      goal_distance,
      LINEAR_SPEED,
      cmd.twist.linear.x,
      cmd.twist.linear.y);

    return cmd;
  }

  // --------------------------------------------------
  // 6. 현재 Path Segment
  // --------------------------------------------------

  const auto & p1 = global_plan_.poses[nearest_index];
  const auto & p2 = global_plan_.poses[nearest_index + 1];

  const double segment_dx =
    p2.pose.position.x - p1.pose.position.x;

  const double segment_dy =
    p2.pose.position.y - p1.pose.position.y;

  RCLCPP_INFO(
    node_->get_logger(),
    "SEGMENT: index=%zu -> %zu dx=%.3f dy=%.3f",
    nearest_index,
    nearest_index + 1,
    segment_dx,
    segment_dy);

  // --------------------------------------------------
  // 7. 코너 확인
  // --------------------------------------------------

  if (nearest_index + 2 < global_plan_.poses.size()) {
    const auto & p3 = global_plan_.poses[nearest_index + 2];

    const double dx1 =
      p2.pose.position.x - p1.pose.position.x;

    const double dy1 =
      p2.pose.position.y - p1.pose.position.y;

    const double dx2 =
      p3.pose.position.x - p2.pose.position.x;

    const double dy2 =
      p3.pose.position.y - p2.pose.position.y;

    const double cross =
      dx1 * dy2 - dy1 * dx2;

    const bool is_corner =
      std::abs(cross) > 0.0001;

    const double corner_dx =
      p2.pose.position.x - robot_x;

    const double corner_dy =
      p2.pose.position.y - robot_y;

    const double corner_distance =
      std::hypot(corner_dx, corner_dy);

    if (is_corner &&
        corner_distance <= 0.08 &&
        nearest_index + 1 != last_corner_index_)
    {
      target_yaw_ = std::atan2(dy2, dx2);
      last_corner_index_ = nearest_index + 1;
      rotating_ = true;

      RCLCPP_INFO(
        node_->get_logger(),
        "Corner detected. Start rotation. corner_index=%zu target_yaw=%.3f",
        nearest_index + 1,
        target_yaw_);

      return cmd;
    }
  }

  // --------------------------------------------------
  // 8. 일반 직선 구간 이동
  // --------------------------------------------------

  const double robot_dx =
    std::cos(robot_yaw) * segment_dx +
    std::sin(robot_yaw) * segment_dy;

  const double robot_dy =
    -std::sin(robot_yaw) * segment_dx +
    std::cos(robot_yaw) * segment_dy;

  RCLCPP_INFO(
    node_->get_logger(),
    "BODY SEGMENT: robot_dx=%.3f robot_dy=%.3f",
    robot_dx,
    robot_dy);

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
  // 9. 직선 주행 중 X/Y축 정렬 유지
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
    normalizeAngle(axis_yaw_1 - robot_yaw);

  const double error_2 =
    normalizeAngle(axis_yaw_2 - robot_yaw);

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
    cmd.twist.angular.z = std::clamp(
      YAW_KP * yaw_error,
      -MAX_ANGULAR_SPEED,
      MAX_ANGULAR_SPEED);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "STRAIGHT ALIGN: robot_yaw=%.3f target_yaw=%.3f yaw_error=%.3f angular=%.3f",
    robot_yaw,
    normalizeAngle(robot_yaw + yaw_error),
    yaw_error,
    cmd.twist.angular.z);

  RCLCPP_INFO(
    node_->get_logger(),
    "MOVE CMD: x=%.3f y=%.3f angular=%.3f",
    cmd.twist.linear.x,
    cmd.twist.linear.y,
    cmd.twist.angular.z);

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