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
    radius_inner_         = this->declare_parameter<double>("radius_inner", 0.8);
    radius_outer_         = this->declare_parameter<double>("radius_outer", 1.0);
    max_linear_speed_  = this->declare_parameter<double>("max_linear_speed", 2.5);
    max_delta_         = this->declare_parameter<double>("max_delta", 0.015);

    max_inner_angular_speed_ = max_linear_speed_ * radius_inner_;
    max_outer_angular_speed_ = max_linear_speed_ * radius_outer_;

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
    double rotate_inner = radius_inner_ * (msg->angular.z / max_inner_angular_speed_);
    double rotate_outer = radius_outer_ * (msg->angular.z / max_outer_angular_speed_);

    // Normalize to [-1, 1]
    double s = std::max(std::abs(move), std::max(std::abs(rotate_inner), std::abs(rotate_outer)));
    if (s > 1.0) {
      move   /= s;
      rotate_inner /= s;
      rotate_outer /= s;
    }

    if (move > 0.0) {
      if (rotate_inner > 0.0) {
        bl_out_  = move - rotate_inner;
        br_out_ = std::max(move, rotate_inner);
        fl_out_  = move - rotate_outer;
        fr_out_ = std::max(move, rotate_outer);
      } else {
        bl_out_  = std::max(move, -rotate_inner);
        br_out_ = move + rotate_inner;
        fl_out_  = std::max(move, -rotate_outer);
        fr_out_ = move + rotate_outer;
      }
    } else {
      if (rotate_outer > 0.0) {
        bl_out_  = -std::max(-move, rotate_inner);
        br_out_ = move + rotate_inner;
        fl_out_  = -std::max(-move, rotate_outer);
        fr_out_ = move + rotate_outer;
      } else {
        bl_out_  = move - rotate_inner;
        br_out_ = -std::max(-move, -rotate_inner);
        fl_out_  = move - rotate_outer;
        fr_out_ = -std::max(-move, -rotate_outer);
      }
    }
  }

  void publishLoop() {
    // Rate limit changes
    double fl_cmd = clampDelta(last_fl_out_,  fl_out_,  max_delta_);
    double fr_cmd = clampDelta(last_fr_out_, fr_out_, max_delta_);
    double bl_cmd = clampDelta(last_bl_out_, bl_out_, max_delta_);
    double br_cmd = clampDelta(last_br_out_, br_out_, max_delta_);

    // ros_phoenix::msg::MotorControl left_msg;
    // left_msg.mode  = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    // left_msg.value = left_cmd;

    // ros_phoenix::msg::MotorControl right_msg;
    // right_msg.mode  = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    // right_msg.value = right_cmd;

    ros_phoenix::msg::MotorControl fl_msg;
    ros_phoenix::msg::MotorControl fr_msg;
    ros_phoenix::msg::MotorControl bl_msg;
    ros_phoenix::msg::MotorControl br_msg;

    fl_msg.mode = fr_msg.mode = bl_msg.mode = br_msg.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    fl_msg.value = fl_cmd;
    fr_msg.value = fr_cmd;
    bl_msg.value = bl_cmd;
    br_msg.value = br_cmd;

    // Publish based on control mode: 0=rear, 1=front, 2=both
    if (control_mode_ == 1) {
      pub_fl_->publish(fl_msg);
      pub_fr_->publish(fr_msg);
    } else if (control_mode_ == 0) {
      pub_bl_->publish(bl_msg);
      pub_br_->publish(br_msg);
    } else {
      pub_fl_->publish(fl_msg);
      pub_bl_->publish(bl_msg);
      pub_fr_->publish(fr_msg);
      pub_br_->publish(br_msg);
    }

    last_fl_out_ = fl_cmd;
    last_fr_out_ = fr_cmd;
    last_bl_out_ = bl_cmd;
    last_br_out_ = br_cmd;
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
  double fl_out_ = 0.0, fr_out_ = 0.0, bl_out_ = 0.0, br_out_ = 0.0;
  double last_fl_out_ = 0.0, last_fr_out_ = 0.0, last_bl_out_ = 0.0, last_br_out_ = 0.0;

  // Params
  double radius_inner_{0.8};
  double radius_outer_{0.8};
  double max_linear_speed_{2.5};
  double max_inner_angular_speed_{radius_inner_ * max_linear_speed_};
  double max_outer_angular_speed_{radius_outer_ * max_linear_speed_};
  double max_delta_{0.015};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FalconMotorController>());
  rclcpp::shutdown();
  return 0;
}