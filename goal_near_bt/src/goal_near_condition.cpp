#include "goal_near_bt/goal_near_condition.hpp"

#include <cmath>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace goal_near_bt
{

GoalNearCondition::GoalNearCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ =
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}


BT::NodeStatus GoalNearCondition::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  double distance_threshold = 0.20;

  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(node_->get_logger(), "GoalNear: goal input missing");
    return BT::NodeStatus::FAILURE;
  }

  getInput("distance", distance_threshold);

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform(
      goal.header.frame_id,
      "base_footprint",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "GoalNear: TF failed: %s",
      ex.what());

    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = transform.transform.translation.x;
  const double robot_y = transform.transform.translation.y;

  const double goal_x = goal.pose.position.x;
  const double goal_y = goal.pose.position.y;

  const double dx = goal_x - robot_x;
  const double dy = goal_y - robot_y;

  const double distance = std::hypot(dx, dy);

  if (distance <= distance_threshold) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace goal_near_bt


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<goal_near_bt::GoalNearCondition>("GoalNear");
}