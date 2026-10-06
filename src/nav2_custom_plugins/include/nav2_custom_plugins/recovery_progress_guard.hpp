#ifndef NAV2_CUSTOM_PLUGINS__RECOVERY_PROGRESS_GUARD_HPP_
#define NAV2_CUSTOM_PLUGINS__RECOVERY_PROGRESS_GUARD_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_custom_plugins
{

class RecoveryProgressGuard : public BT::DecoratorNode
{
public:
  RecoveryProgressGuard(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 0.08, "Distance to consider recovery successful"),
      BT::InputPort<double>("timeout", 3.0, "Time allowed to reach the distance before failing"),
      BT::InputPort<std::string>("global_frame", "odom", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint", "Robot base frame")
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  bool getRobotPose(double & x, double & y);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  
  bool initialized_{false};
  double start_x_{0.0};
  double start_y_{0.0};
  rclcpp::Time start_time_;
  
  double distance_{0.08};
  double timeout_{3.0};
  std::string global_frame_{"odom"};
  std::string robot_base_frame_{"base_footprint"};
};

}  // namespace nav2_custom_plugins

#endif  // NAV2_CUSTOM_PLUGINS__RECOVERY_PROGRESS_GUARD_HPP_
