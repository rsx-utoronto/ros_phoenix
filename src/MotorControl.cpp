#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "ros_phoenix/TalonNode.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using ros_phoenix::TalonNode;
using std::chrono::milliseconds;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros_phoenix");

  // Parameters
  const auto can_interface = node->declare_parameter<std::string>("can_interface", "can0");
  const auto talon_names = node->declare_parameter<std::vector<std::string>>("talons", {});

  // Set CAN interface for CTRE Phoenix
  ctre::phoenix::platform::can::SetCANInterface(can_interface.c_str());

  // Build Talons from parameters
  std::vector<std::unique_ptr<TalonNode>> talons;
  talons.reserve(talon_names.size());

  for (const auto& name : talon_names) {
    const std::string pfx = "talons." + name + ".";

    int id = node->declare_parameter<int>(pfx + "id", -1);
    if (id < 0) {
      RCLCPP_WARN(node->get_logger(), "Skipping Talon '%s': missing/invalid id", name.c_str());
      continue;
    }

    ros_phoenix::msg::TalonConfig cfg{};
    cfg.inverted          = node->declare_parameter<bool>(pfx + "inverted", false);
    cfg.peak_voltage      = node->declare_parameter<double>(pfx + "peak_voltage", 12.0);
    cfg.pot               = node->declare_parameter<bool>(pfx + "pot", false);
    cfg.invert_sensor     = node->declare_parameter<bool>(pfx + "invert_sensor", false);
    cfg.p                 = node->declare_parameter<double>(pfx + "p", 0.0);
    cfg.i                 = node->declare_parameter<double>(pfx + "i", 0.0);
    cfg.d                 = node->declare_parameter<double>(pfx + "d", 0.0);
    cfg.f                 = node->declare_parameter<double>(pfx + "f", 0.0);
    cfg.cont_current      = node->declare_parameter<int>(pfx + "cont_current", 0);
    cfg.peak_current_dur  = node->declare_parameter<int>(pfx + "peak_current_dur", 0);
    cfg.brake_mode        = node->declare_parameter<bool>(pfx + "brake_mode", true);

    talons.emplace_back(std::make_unique<TalonNode>(node, name, id, cfg));
    RCLCPP_INFO(node->get_logger(), "Created Talon '%s' id=%d", name.c_str(), id);
  }

  // Keep CTRE enabled (similar to ROS 1 Unmanaged::FeedEnable loop)
  auto feed_timer = node->create_wall_timer(
      milliseconds(20),
      [] { ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100); });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
