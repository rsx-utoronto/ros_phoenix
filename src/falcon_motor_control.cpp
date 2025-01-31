#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros_phoenix/MotorControl.h"
#include <cmath>
#include <sensor_msgs/Joy.h>

static double leftMotorOutput = 0.0;
static double rightMotorOutput = 0.0;
static bool invertLeft = false;
static bool invertRight = true;
static double RADIUS=0.8;
static double MAX_LINEAR_SPEED=2.5;
static double MAX_ANGULAR_SPEED=MAX_LINEAR_SPEED*RADIUS;

static double controlMode = 2; // 0 = rear wheel, 1 = front wheel, 2 = both

// update controller mode (front wheel, rear wheel, or both)
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // indexs for controller values
    int rear_wheel = 0; // rear wheel drive
    int front_wheel = 2; // front wheel drive

    if (joy->buttons[rear_wheel] == 1) {
        controlMode = 0;
    } else if (joy->buttons[front_wheel] == 1) {
        controlMode = 1;
    } else { // both front and back are on
        controlMode = 2;
    }
} 

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double moveValue = msg->linear.x/MAX_LINEAR_SPEED;
    double rotateValue = RADIUS*(msg->angular.z/MAX_ANGULAR_SPEED);

    if(std::abs(moveValue)>1 || std::abs(rotateValue)>1) {
        if(std::abs(moveValue)>std::abs(rotateValue)) {
            moveValue=moveValue/std::abs(moveValue);
            rotateValue=rotateValue/std::abs(moveValue);
        } else {
            moveValue=moveValue/std::abs(rotateValue);
            rotateValue=rotateValue/std::abs(rotateValue);
        }
    }

    if (moveValue > 0.0) {
        if (rotateValue > 0.0) {
            leftMotorOutput = moveValue - rotateValue;
            rightMotorOutput = std::max(moveValue, rotateValue);
        } else {
            leftMotorOutput = std::max(moveValue, -rotateValue);
            rightMotorOutput = moveValue + rotateValue;
        }
    } else {
        if (rotateValue > 0.0) {
            leftMotorOutput = -std::max(-moveValue, rotateValue);
            rightMotorOutput = moveValue + rotateValue;
        } else {
            leftMotorOutput = moveValue - rotateValue;
            rightMotorOutput = -std::max(-moveValue, -rotateValue);
        }
    }
    ROS_INFO("Move=%f Rotate=%f", moveValue, rotateValue);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "falcon_motor_controller");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("drive", 1, cmdCallback);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallback);

    ros::Publisher fl = nh.advertise<ros_phoenix::MotorControl>("/front_left/set", 1);
    ros::Publisher fr = nh.advertise<ros_phoenix::MotorControl>("/front_right/set", 1);
    ros::Publisher bl = nh.advertise<ros_phoenix::MotorControl>("/back_left/set", 1);
    ros::Publisher br = nh.advertise<ros_phoenix::MotorControl>("/back_right/set", 1);

    ros::Rate loop_rate(50);
    while (ros::ok()) {

        ros_phoenix::MotorControlPtr left(new ros_phoenix::MotorControl);
        left->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        std::cout << "left: " << leftMotorOutput<< std::endl;
        left->value = leftMotorOutput;

        if (controlMode == 1) { // front wheel drive
            fl.publish(left);
            std::cout << "front wheel" << std::endl;
        } else if (controlMode == 0) { // back wheel drive
            std::cout << "back wheel" << std::endl;
            bl.publish(left);
        } else { // both front and back
            std::cout << "both" << std::endl;
            fl.publish(left);
            bl.publish(left);
        }

        ros_phoenix::MotorControlPtr right(new ros_phoenix::MotorControl);
        right->mode = ros_phoenix::MotorControl::PERCENT_OUTPUT;
        std::cout << "right: " << rightMotorOutput << std::endl;
        right->value = rightMotorOutput;

        if (controlMode == 1) { // front wheel drive
            std::cout << "front wheel" << std::endl;
            fr.publish(right);
        } else if (controlMode == 0) { // back wheel drive
            std::cout << "back wheel" << std::endl;
            br.publish(right);
        } else { // both front and back
            std::cout << "both" << std::endl;
            fr.publish(right);
            br.publish(right);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
