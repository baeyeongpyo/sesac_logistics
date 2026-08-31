#ifndef ORTHOGONAL_PLANNER__ORTHOGONAL_PLANNER_HPP_
#define ORTHOGONAL_PLANNER__ORTHOGONAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace orthogonal_planner
{

class OrthogonalPlanner : public nav2_core::GlobalPlanner
{
public:
  OrthogonalPlanner() = default;
  ~OrthogonalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  nav2_costmap_2d::Costmap2D * costmap_;
};

}  // namespace orthogonal_planner

#endif  // ORTHOGONAL_PLANNER__ORTHOGONAL_PLANNER_HPP_