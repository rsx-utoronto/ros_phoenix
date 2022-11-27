#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros_phoenix/MotorControl.h"
#include <cmath> 

static double leftMotorOutput = 0.0;
static double rightMotorOutput = 0.0;
static bool invertLeft = false;
static bool invertRight = true;
static double MAX_LINEAR_SPEED=0.6;
static double MAX_ANGULAR_SPEED=0.4;
static double RADIUS=1;

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double moveValue = msg->linear.x/MAX_LINEAR_SPEED;
    double rotateValue = RADIUS*(msg->angular.z/MAX_ANGULAR_SPEED);
    // if(std::abs(moveValue)>1 || std::abs(rotateValue)>1) {
    //     if(std::abs(moveValue)>std::abs(rotateValue)) {
    //         moveValue=moveValue/std::abs(moveValue);
    //         rotateValue=rotateValue/std::abs(moveValue);
    //     } else {
    //         moveValue=moveValue/std::abs(rotateValue);
    //         rotateValue=rotateValue/std::abs(rotateValue);
    //     }
    // }

    // if (moveValue > 0.0) {
    //     if (rotateValue > 0.0) {
    //         leftMotorOutput = moveValue - rotateValue;
    //         rightMotorOutput = std::max(moveValue, rotateValue);
    //     } else {
    //         leftMotorOutput = std::max(moveValue, -rotateValue);
    //         rightMotorOutput = moveValue + rotateValue;
    //     }
    // } else {
    //     if (rotateValue > 0.0) {
    //         leftMotorOutput = -std::max(-moveValue, rotateValue);
    //         rightMotorOutput = moveValue + rotateValue;
    //     } else {
    //         leftMotorOutput = moveValue - rotateValue;
    //         rightMotorOutput = -std::max(-moveValue, -rotateValue);
    //     }
    // }

    // other option 
    // Changes to test: 
    //  - instead scale by the turn value 
    if (rotateValue < 0.0) {
        rightMotorOutput = -moveValue;
        leftMotorOutput = moveValue; 
    } else if (rotateValue > 0.0) {
        rightMotorOutput = moveValue;
        leftMotorOutput = -moveValue;
    } else {
        rightMotorOutput = moveValue;
        leftMotorOutput = moveValue;
    }
    ROS_INFO("Move=%f Rotate=%f", moveValue, rotateValue);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "falcon_motor_controller");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("drive", 1, cmdCallback);

    ros::Publisher fl = nh.advertise<ros_phoenix::MotorControl>("/front_left/set", 1);
    ros::Publisher fr = nh.advertise<ros_phoenix::MotorControl>("/front_right/set", 1);
    ros::Publisher ml = nh.advertise<ros_phoenix::MotorControl>("/mid_left/set", 1);
    ros::Publisher mr = nh.advertise<ros_phoenix::MotorControl>("/mid_right/set", 1);
    ros::Publisher bl = nh.advertise<ros_phoenix::MotorControl>("/back_left/set", 1);
    ros::Publisher br = nh.advertise<ros_phoenix::MotorControl>("/back_right/set", 1);

    // ros::Subscriber mod = nh.subscribe("joy", 1, cmdCallback);
    float power_limit = 1; // default power_limit

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        // different modes
        // if (mod.buttons[15] == 1){float power_limit = 0.1;}
        // else if (mod.buttons[13] == 1){float power_limit = 0.25;}
        // else if (mod.buttons[16] == 1){float power_limit = 0.5;}
        // else if (mod.buttons[14] == 1){float power_limit = 0.75;}
    

        ros_phoenix::MotorControlPtr left(new ros_phoenix::MotorControl);
        left->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        left->value = leftMotorOutput*power_limit;
        fl.publish(left);
        ml.publish(left);
        bl.publish(left);

        ros_phoenix::MotorControlPtr right(new ros_phoenix::MotorControl);
        right->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        right->value = rightMotorOutput*power_limit;
        fr.publish(right);
        mr.publish(right);
        br.publish(right);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
