#include <chrono>
#include <utility>

#include "ros_phoenix/TalonNode.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace ros_phoenix {

TalonNode::TalonNode(const rclcpp::Node::SharedPtr& node,
                     const std::string& name,
                     int id,
                     const ros_phoenix::msg::TalonConfig& config)
  : node_(node)
  , name_(name)
  , config_(config)
  , talon(id)
  , last_update_(node_->get_clock()->now())
  , control_mode_(ControlMode::PercentOutput)
  , output_(0.0)
  , disabled_(false)
  , configured_(false)
  , not_configured_warned_(false)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // Topics relative to the node’s namespace (same as ROS 1: "status" and "set")
  status_pub_ = node_->create_publisher<ros_phoenix::msg::MotorStatus>("status", rclcpp::QoS(10));
  set_sub_ = node_->create_subscription<ros_phoenix::msg::MotorControl>(
      "set", rclcpp::QoS(10),
      std::bind(&TalonNode::set, this, std::placeholders::_1));

  talon.NeutralOutput();

  // Dynamic parameter updates for this talon (prefix: talons.<name_>.)
  const std::string pfx = "talons." + name_ + ".";
  param_cb_handle_ = node_->add_on_set_parameters_callback(
    [this, pfx](const std::vector<rclcpp::Parameter> & params) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      ros_phoenix::msg::TalonConfig new_cfg = config_;
      bool changed = false;

      for (const auto & p : params) {
        const auto & key = p.get_name();
        if      (key == pfx + "inverted")          { new_cfg.inverted = p.as_bool(); changed = true; }
        else if (key == pfx + "peak_voltage")      { new_cfg.peak_voltage = p.as_double(); changed = true; }
        else if (key == pfx + "pot")               { new_cfg.pot = p.as_bool(); changed = true; }
        else if (key == pfx + "invert_sensor")     { new_cfg.invert_sensor = p.as_bool(); changed = true; }
        else if (key == pfx + "p")                 { new_cfg.p = p.as_double(); changed = true; }
        else if (key == pfx + "i")                 { new_cfg.i = p.as_double(); changed = true; }
        else if (key == pfx + "d")                 { new_cfg.d = p.as_double(); changed = true; }
        else if (key == pfx + "f")                 { new_cfg.f = p.as_double(); changed = true; }
        else if (key == pfx + "cont_current")      { new_cfg.cont_current = p.as_int(); changed = true; }
        else if (key == pfx + "peak_current_dur")  { new_cfg.peak_current_dur = p.as_int(); changed = true; }
        else if (key == pfx + "brake_mode")        { new_cfg.brake_mode = p.as_bool(); changed = true; }
      }

      if (changed) {
        this->reconfigure(new_cfg);
        result.successful = true;
        result.reason = "applied talon reconfigure";
      } else {
        result.successful = true;
      }
      return result;
    });

  talon.NeutralOutput();

  // Optional: periodic update loop (50 Hz)
  update_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&TalonNode::update, this));
}

void TalonNode::set(const ros_phoenix::msg::MotorControl::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  control_mode_ = static_cast<ControlMode>(msg->mode);
  output_ = msg->value;
  last_update_ = node_->get_clock()->now();
}

void TalonNode::reconfigure(const ros_phoenix::msg::TalonConfig& cfg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_INFO(node_->get_logger(), "Reconfigure called on %s", name_.c_str());
  config_ = cfg;
  configured_ = false;
}

void TalonNode::configure()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (talon.GetFirmwareVersion() == -1) {
    if (!not_configured_warned_) {
      RCLCPP_WARN(node_->get_logger(), "Talon hasn't been seen: %d", talon.GetDeviceID());
      not_configured_warned_ = true;
    }
    return;
  }

  TalonSRXConfiguration c;
  SlotConfiguration slot;
  slot.kP = config_.p;
  slot.kI = config_.i;
  slot.kD = config_.d;
  slot.kF = config_.f;
  c.slot0 = slot;
  c.voltageCompSaturation = config_.peak_voltage;
  c.continuousCurrentLimit = config_.cont_current;
  c.peakCurrentLimit = config_.cont_current;
  c.peakCurrentDuration = config_.peak_current_dur;
  c.pulseWidthPeriod_EdgesPerRot = 4096;

  ErrorCode error = talon.ConfigAllSettings(c, 10);

  if (error != ErrorCode::OK) {
    if (!not_configured_warned_) {
      RCLCPP_WARN(node_->get_logger(), "Reconfiguring Talon %s %d failed!", name_.c_str(), talon.GetDeviceID());
      not_configured_warned_ = true;
    }
    configured_ = false;
    return;
  }

  configureStatusPeriod();

  if (config_.pot) {
    talon.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::Analog);
  } else {
    talon.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);
  }

  talon.EnableCurrentLimit(true);
  talon.SetSensorPhase(config_.invert_sensor);
  talon.SelectProfileSlot(0, 0);
  talon.SetInverted(config_.inverted);
  talon.EnableVoltageCompensation(true);

  talon.SetNeutralMode(config_.brake_mode ? NeutralMode::Brake : NeutralMode::Coast);

  RCLCPP_INFO(node_->get_logger(),
              "Reconfigured Talon: %s with id=%d p=%.3f i=%.3f d=%.3f",
              name_.c_str(), talon.GetDeviceID(), config_.p, config_.i, config_.d);

  configured_ = true;
  not_configured_warned_ = false;
}

void TalonNode::update()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // Configure if needed
  if (!configured_) {
    configure();
  }

  // Watchdog: disable if no recent commands
  auto now = node_->get_clock()->now();
  if ((now - last_update_) > rclcpp::Duration::from_seconds(0.2)) {
    if (!disabled_) {
      RCLCPP_WARN(node_->get_logger(), "Talon disabled for not receiving updates: %s", name_.c_str());
    }
    disabled_ = true;
    control_mode_ = ControlMode::PercentOutput;
    output_ = 0.0;
  } else {
    if (disabled_) {
      RCLCPP_INFO(node_->get_logger(), "Talon re-enabled for receiving updates: %s", name_.c_str());
    }
    disabled_ = false;
  }

  // Command output
  if (output_ == 0.0 || !configured_) {
    talon.NeutralOutput();
  } else {
    talon.Set(control_mode_, output_);
  }

  // Publish status
  ros_phoenix::msg::MotorStatus status;
  status.temperature = talon.GetTemperature();
  status.bus_voltage = talon.GetBusVoltage();

  status.output_percent = talon.GetMotorOutputPercent();
  status.output_voltage = talon.GetMotorOutputVoltage();
  status.output_current = talon.GetOutputCurrent();

  status.position = talon.GetSelectedSensorPosition();
  status.velocity = talon.GetSelectedSensorVelocity() * 10.0; // per-100ms -> per-sec

  status.fwd_limit = talon.GetSensorCollection().IsFwdLimitSwitchClosed();
  status.rev_limit = talon.GetSensorCollection().IsRevLimitSwitchClosed();

  status_pub_->publish(status);
}

void TalonNode::configureStatusPeriod()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 20);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 20);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 50);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 100);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 20);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 20);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 20);
  talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 100);
}

} // namespace ros_phoenix
