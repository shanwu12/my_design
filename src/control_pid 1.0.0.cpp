#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <std_msgs/UInt8.h>
#include "PID_lib/pid.h"

using namespace std;
using namespace ros;
using namespace DJI::onboardSDK;

ros::Publisher ctrl_vel_pub;
ros::Subscriber odometry_sub;
ros::Publisher guidance_bias_pub;

DJIDrone* drone;

PID *ctrl_x;
PID *ctrl_y;
PID *ctrl_z;
PID *ctrl_yaw;

double Kp_pos;
double Ki_pos;
double Kd_pos;

double Kp_height;
double Ki_height;
double Kd_height;

double Kp_yaw;
double Ki_yaw;
double Kd_yaw;

float ctrl_data[4] = {0,0,0,0};
float target_position[3] = {0,0,2};
float target_yaw = 0;

float dt = 0.01;
float first_time = 0.0;

double pid_ctrl_limit = 0.2;
double pid_ctrl_limit_vert = 0.2;
double pid_yaw_limit = 5;


void guidance_odometry_callback(const nav_msgs::Odometry& guidance_odometry) {
    /* Publish the output control velocity from PID controller */
    dt = guidance_odometry.header.stamp.toSec() - first_time;
    first_time = guidance_odometry.header.stamp.toSec();

//    ctrl_x=
//    ctrl_x->set_point();

    geometry_msgs::Vector3 velocity;

    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.2;
    cout<<"control_pid"<<endl;
    ctrl_vel_pub.publish(velocity);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_pid");
    ros::NodeHandle nh;
    std_msgs::UInt8 bias_correct_msg;

    drone = new DJIDrone(nh);
    ros::Rate loop_rate(48);

    odometry_sub = nh.subscribe("/guidance/odometry", 1, guidance_odometry_callback);

    ctrl_vel_pub = nh.advertise<geometry_msgs::Vector3>("/control_velocity", 20);
    guidance_bias_pub=nh.advertise<std_msgs::UInt8>("/guidance/bias",20);

    cout << "init success!"<<endl;
    bias_correct_msg.data = 1;

    while(ros::ok())
    {
        guidance_bias_pub.publish(bias_correct_msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
