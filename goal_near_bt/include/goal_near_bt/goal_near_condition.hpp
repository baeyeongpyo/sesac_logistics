#ifndef GOAL_NEAR_BT__GOAL_NEAR_CONDITION_HPP_
#define GOAL_NEAR_BT__GOAL_NEAR_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace goal_near_bt
{

class GoalNearCondition : public BT::ConditionNode
{
public:
  GoalNearCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GoalNearCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<double>("distance", 0.20, "Goal distance threshold")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace goal_near_bt

#endif  // GOAL_NEAR_BT__GOAL_NEAR_CONDITION_HPP_