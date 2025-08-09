#ifndef ROS_PHOENIX_TALONNODE_H
#define ROS_PHOENIX_TALONNODE_H

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "ros_phoenix/msg/talon_config.hpp"  // Update this header to be ROS 2 friendly (no dynamic_reconfigure)
#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <string>

namespace ros_phoenix {

class TalonNode {
private:
  // Thread-safety
  std::recursive_mutex mutex_;

  // ROS 2 node
  rclcpp::Node::SharedPtr node_;
  std::string name_;

  // Config (replace dynamic_reconfigure with parameters or a plain struct)
  ros_phoenix::msg::TalonConfig config_;

  // CTRE device
  TalonSRX talon;

  // Pub/Sub
  rclcpp::Publisher<ros_phoenix::msg::MotorStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<ros_phoenix::msg::MotorControl>::SharedPtr set_sub_;

  // Timing and state
  rclcpp::Time last_update_;
  ControlMode control_mode_;
  double output_;
  bool disabled_;
  bool configured_;
  bool not_configured_warned_;

  // Optional: update loop timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Optional: parameter callback (ROS 2 replacement for dynamic_reconfigure)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

public:
  TalonNode(const rclcpp::Node::SharedPtr& node,
            const std::string& name,
            int id,
            const ros_phoenix::msg::TalonConfig& config);

  TalonNode& operator=(const TalonNode&) = delete;
  ~TalonNode() = default;

  // Reconfigure without dynamic_reconfigure (apply new config directly or via params)
  void reconfigure(const ros_phoenix::msg::TalonConfig& config);

  // Apply current config to hardware
  void configure();

  // ROS 2 message callback
  void set(const ros_phoenix::msg::MotorControl::SharedPtr msg);

  // Periodic update (publish status, watchdog, etc.)
  void update();

  void configureStatusPeriod();
};

} // namespace ros_phoenix

#endif // ROS_PHOENIX_TALONNODE_H
