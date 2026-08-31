#ifndef ORTHOGONAL_CONTROLLER__ORTHOGONAL_CONTROLLER_HPP_
#define ORTHOGONAL_CONTROLLER__ORTHOGONAL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.h"

namespace orthogonal_controller
{

class OrthogonalController : public nav2_core::Controller
{
public:
  OrthogonalController() = default;
  ~OrthogonalController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(
    const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(
    const double & speed_limit,
    const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  nav2_costmap_2d::Costmap2D * costmap_;

  nav_msgs::msg::Path global_plan_;

  bool rotating_ = false;

  double target_yaw_ = 0.0;

  std::size_t last_corner_index_ =
    std::numeric_limits<std::size_t>::max();
};

}  // namespace orthogonal_controller

#endif  // ORTHOGONAL_CONTROLLER__ORTHOGONAL_CONTROLLER_HPP_