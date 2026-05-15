#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "ros_phoenix/msg/motor_control.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class FalconMotorController : public rclcpp::Node {
public:
  FalconMotorController() : rclcpp::Node("falcon_motor_controller") {
    // Params (tunable)
    radius_            = this->declare_parameter<double>("radius", 0.8);
    max_linear_speed_  = this->declare_parameter<double>("max_linear_speed", 2.5);
    max_delta_         = this->declare_parameter<double>("max_delta", 0.02);

    max_angular_speed_ = max_linear_speed_ * radius_;

    // Subs
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "drive", 1, std::bind(&FalconMotorController::cmdCallback, this, _1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&FalconMotorController::joyCallback, this, _1));

    // Pubs (kept same topic names as ROS 1; remove leading '/' if you prefer namespaced topics)
    pub_fl_ = this->create_publisher<ros_phoenix::msg::MotorControl>("/front_left/set", 1);
    pub_fr_ = this->create_publisher<ros_phoenix::msg::MotorControl>("/front_right/set", 1);
    pub_bl_ = this->create_publisher<ros_phoenix::msg::MotorControl>("/back_left/set", 1);
    pub_br_ = this->create_publisher<ros_phoenix::msg::MotorControl>("/back_right/set", 1);

    // 50 Hz timer to publish
    timer_ = this->create_wall_timer(20ms, std::bind(&FalconMotorController::publishLoop, this));
  }

private:
  // Callbacks
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    // 0: rear, 2: front; else both
    const int rear_wheel_btn = 0;
    const int front_wheel_btn = 2;

    if (rear_wheel_btn < static_cast<int>(joy->buttons.size()) && joy->buttons[rear_wheel_btn] == 1) {
      control_mode_ = 0;
    } else if (front_wheel_btn < static_cast<int>(joy->buttons.size()) && joy->buttons[front_wheel_btn] == 1) {
      control_mode_ = 1;
    } else {
      control_mode_ = 2;
    }
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double move = msg->linear.x / max_linear_speed_;
    double rotate = radius_ * (msg->angular.z / max_angular_speed_);

    // Normalize to [-1, 1]
    double s = std::max(std::abs(move), std::abs(rotate));
    if (s > 1.0) {
      move   /= s;
      rotate /= s;
    }

    if (move > 0.0) {
      if (rotate > 0.0) {
        left_out_  = move - rotate;
        right_out_ = std::max(move, rotate);
      } else {
        left_out_  = std::max(move, -rotate);
        right_out_ = move + rotate;
      }
    } else {
      if (rotate > 0.0) {
        left_out_  = -std::max(-move, rotate);
        right_out_ = move + rotate;
      } else {
        left_out_  = move - rotate;
        right_out_ = -std::max(-move, -rotate);
      }
    }
  }

  void publishLoop() {
    // Rate limit changes
    double left_cmd  = clampDelta(last_left_out_,  left_out_,  max_delta_);
    double right_cmd = clampDelta(last_right_out_, right_out_, max_delta_);

    ros_phoenix::msg::MotorControl left_msg;
    left_msg.mode  = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    left_msg.value = left_cmd;

    ros_phoenix::msg::MotorControl right_msg;
    right_msg.mode  = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    right_msg.value = right_cmd;

    // Publish based on control mode: 0=rear, 1=front, 2=both
    if (control_mode_ == 1) {
      pub_fl_->publish(left_msg);
      pub_fr_->publish(right_msg);
    } else if (control_mode_ == 0) {
      pub_bl_->publish(left_msg);
      pub_br_->publish(right_msg);
    } else {
      pub_fl_->publish(left_msg);
      pub_bl_->publish(left_msg);
      pub_fr_->publish(right_msg);
      pub_br_->publish(right_msg);
    }

    last_left_out_  = left_cmd;
    last_right_out_ = right_cmd;
  }

  static double clampDelta(double last, double target, double max_delta) {
    const double diff = target - last;
    if (std::abs(diff) <= max_delta) return target;
    return last + std::copysign(max_delta, diff);
  }

  // Members
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr pub_fl_, pub_fr_, pub_bl_, pub_br_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  int control_mode_ = 2; // 0=rear, 1=front, 2=both
  double left_out_ = 0.0, right_out_ = 0.0;
  double last_left_out_ = 0.0, last_right_out_ = 0.0;

  // Params
  double radius_{0.8};
  double max_linear_speed_{2.5};
  double max_angular_speed_{radius_ * max_linear_speed_};
  double max_delta_{0.02};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FalconMotorController>());
  rclcpp::shutdown();
  return 0;
}