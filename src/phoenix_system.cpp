#include "ros_phoenix/phoenix_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace ros_phoenix {

PhoenixSystem::InterfaceType PhoenixSystem::str_to_interface(const std::string& str)
{
    if (str == "percent_output") {
        return InterfaceType::PERCENT_OUTPUT;

    } else if (str == hardware_interface::HW_IF_POSITION) {
        return InterfaceType::POSITION;

    } else if (str == hardware_interface::HW_IF_VELOCITY) {
        return InterfaceType::VELOCITY;

    } else {
        return InterfaceType::INVALID;
    }
}

PhoenixSystem::PhoenixSystem()
    : logger_(rclcpp::get_logger("PhoenixSystem"))
{
}

hardware_interface::return_type PhoenixSystem::configure(
    const hardware_interface::HardwareInfo& info)
{
    this->logger_ = rclcpp::get_logger(info.name);
    this->info_ = info;

    if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }

    for (auto joint : info.joints) {
        auto cmd = std::make_shared<ros_phoenix::msg::MotorControl>();

        if (joint.command_interfaces.size() != 1) {
            RCLCPP_ERROR(this->logger_, "Joint '%s' has %d command interfaces. Expected 1.",
                joint.name, joint.command_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }
        InterfaceType cmd_interface = str_to_interface(joint.command_interfaces[0].name);
        if (cmd_interface == InterfaceType::INVALID) {
            RCLCPP_ERROR(this->logger_, "Joint '%s' has an invalid command interface: %s",
                joint.name, joint.command_interfaces[0].name);
            return hardware_interface::return_type::ERROR;
        }
        cmd->mode = cmd_interface;

        for (auto state_inter : joint.state_interfaces) {
            if (str_to_interface(state_inter.name) == InterfaceType::INVALID) {
                RCLCPP_ERROR(this->logger_, "Joint '%s' has an invalid state interface: %s",
                    joint.name, state_inter.name);
                return hardware_interface::return_type::ERROR;
            }
        }

        this->hw_cmd_.push_back(cmd);
        this->hw_status_.push_back(std::make_unique<ros_phoenix::msg::MotorStatus>());
    }

    RCLCPP_INFO(this->logger_, "Finished configure()");
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> PhoenixSystem::export_state_interfaces()
{
    RCLCPP_INFO(this->logger_, "export_state_interfaces()");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < this->info_.joints.size(); i++) {
        for (auto state_inter : this->info_.joints[i].state_interfaces) {
            if (state_inter.name == "percent_output")
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(this->info_.joints[i].name, "percent_output",
                        &(hw_status_[i]->output_percent)));

            else if (state_inter.name == "position")
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    this->info_.joints[i].name, "position", &(hw_status_[i]->position)));

            else if (state_inter.name == "velocity")
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    this->info_.joints[i].name, "velocity", &(hw_status_[i]->velocity)));
        }
    }
    RCLCPP_INFO(this->logger_, "export_state_interfaces()");
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PhoenixSystem::export_command_interfaces()
{
    RCLCPP_INFO(this->logger_, "export_command_interfaces()");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < this->info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(this->info_.joints[i].name,
                this->info_.joints[i].command_interfaces[0].name, &(hw_cmd_[i]->value)));
    }
    RCLCPP_INFO(this->logger_, "export_command_interfaces()");
    return command_interfaces;
}

hardware_interface::return_type PhoenixSystem::start()
{
    RCLCPP_INFO(this->logger_, "start()");
    this->node_ = rclcpp::Node::make_unique(this->info_.name);

    for (auto i = 0u; i < this->info_.joints.size(); i++) {
        this->publishers_.push_back(this->node_->create_publisher<ros_phoenix::msg::MotorControl>(
            this->info_.joints[i].name + "/set", 1));

        this->subscribers_.push_back(
            this->node_->create_subscription<ros_phoenix::msg::MotorStatus>(
                this->info_.joints[i].name + "/status", 1,
                [this, i](const ros_phoenix::msg::MotorStatus::SharedPtr status) {
                    *(this->hw_status_[i]) = *status;
                }));
    }
    RCLCPP_INFO(this->logger_, "start()");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PhoenixSystem::stop()
{
    RCLCPP_INFO(this->logger_, "stop()");
    this->publishers_.clear();
    this->subscribers_.clear();
    this->node_.release();
    RCLCPP_INFO(this->logger_, "stop()");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PhoenixSystem::read()
{
    // Do nothing because this->hw_status_ is updated asynchronously
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PhoenixSystem::write()
{
    for (auto i = 0u; i < this->info_.joints.size(); i++) {
        this->publishers_[i]->publish(*(this->hw_cmd_[i]));
    }
    return hardware_interface::return_type::OK;
}

} // namespace ros_phoenix

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros_phoenix::PhoenixSystem, hardware_interface::SystemInterface)