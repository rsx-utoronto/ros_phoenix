#ifndef ROS_PHOENIX_PHOENIX_BRIDGE
#define ROS_PHOENIX_PHOENIX_BRIDGE

#include <thread>

#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

// #if __has_include("hardware_interface/components/base_interface.hpp")
//   #include "hardware_interface/components/base_interface.hpp"
// #else
//   #include "hardware_interface/base_interface.hpp"
// #endif
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros_phoenix {

class PhoenixBridge
    : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    // std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    RCLCPP_SHARED_PTR_DEFINITIONS(PhoenixBridge)

    PhoenixBridge();

    PhoenixBridge(PhoenixBridge&& other) = default;

    ~PhoenixBridge() = default;

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info);

    std::vector<hardware_interface::StateInterface> export_state_interfaces();

    std::vector<hardware_interface::CommandInterface> export_command_interfaces();

    hardware_interface::return_type start();

    hardware_interface::return_type stop();

    hardware_interface::return_type read();

    hardware_interface::return_type write();

private:
    enum InterfaceType {
        INVALID = -1,
        PERCENT_OUTPUT = 0,
        POSITION = 1,
        VELOCITY = 2,
    };

    static InterfaceType str_to_interface(const std::string& str);

    rclcpp::Logger logger_;

    hardware_interface::HardwareInfo info_;

    std::vector<ros_phoenix::msg::MotorControl::SharedPtr> hw_cmd_;
    std::vector<ros_phoenix::msg::MotorStatus::SharedPtr> hw_status_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread spin_thread_;

    std::vector<rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr> publishers_;
    std::vector<rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr> subscribers_;
};

} // namespace ros_phoenix
#endif // ROS_PHOENIX_PHOENIX_BRIDGE