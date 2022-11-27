#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros_phoenix/MotorControl.h"
#include <cmath> 

static double MotorOutput = 0.0;
static double MAX_LINEAR_SPEED=0.6;
static double MAX_ANGULAR_SPEED=0.1;
static double RADIUS=1;

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //double moveValue = msg->linear.x;
    //double rotateValue = msg->angular.z;

    std::cout << "TWIST MESSAGE: " << msg->linear.x << std::endl;

    double moveValue = msg->linear.x/MAX_LINEAR_SPEED;
    double rotateValue = msg->angular.z/MAX_ANGULAR_SPEED;

    if (moveValue > 0.0) {
        MotorOutput = moveValue;
        // if (rotateValue > 0.0) {
        //     MotorOutput = moveValue;
        // } else {
        //     MotorOutput = std::max(moveValue, -rotateValue);
        // }
    } else {
        MotorOutput = 0.0;
        // if (rotateValue > 0.0) {
        //     MotorOutput = -std::max(-moveValue, rotateValue);
        // } else {
        //     MotorOutput = moveValue - rotateValue;
        // }
    }
    ROS_INFO("Move=%f Rotate=%f", moveValue, rotateValue);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "falcon_motor_controller_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("drive", 10, cmdCallback);

    ros::Publisher m = nh.advertise<ros_phoenix::MotorControl>("/motor/set", 1);

    // ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros_phoenix::MotorControlPtr motor(new ros_phoenix::MotorControl);
        motor->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        motor->value = MotorOutput;
        m.publish(motor);

        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}
