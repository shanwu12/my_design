#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <std_msgs/UInt8.h>
#include "PID_lib/pid.h"
#include "demo_flight/pid_ctrl_data.h"

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

double Kp_pos = 0.8;
double Ki_pos = 0.0;
double Kd_pos = 0.0;

double Kp_height = 0.5;
double Ki_height = 0.0;
double Kd_height = 0.0;

double Kp_yaw = 0.2;
double Ki_yaw = 0.0;
double Kd_yaw = 0.0;

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

//    ctrl_data[0] = target_position[0];
//    ctrl_x->set_point(target_position[0]);
//    ctrl_y->set_point(target_position[1]);
//    ctrl_z->set_point(target_position[1]);
//    ctrl_yaw->set_point(0);

    ctrl_data[0] = ctrl_x -> update(guidance_odometry.pose.pose.position.x, dt);
    ctrl_data[1] = ctrl_y -> update(guidance_odometry.pose.pose.position.y, dt);
    ctrl_data[2] = target_position[2]; // position control logic for z-axis
    ctrl_data[3] = 0; //yaw
 //   ROS_INFO("ctrl_x=%f",ctrl_x);
    ROS_INFO("ctrl_x=%f",ctrl_data[0]);

    if (ctrl_data[0] > pid_ctrl_limit)
        ctrl_data[0] = pid_ctrl_limit;
    if (ctrl_data[0] < -pid_ctrl_limit)
        ctrl_data[0] = -pid_ctrl_limit;
    //ctrl_data[0] = ctrl_data[0] > pid_ctrl_limit ?

    if (ctrl_data[1] > pid_ctrl_limit)
        ctrl_data[1] = pid_ctrl_limit;
    if (ctrl_data[1] < -pid_ctrl_limit)
        ctrl_data[1] = -pid_ctrl_limit;

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

    ctrl_x = new PID( Kp_pos, Ki_pos, Kd_pos, -5, 5, -pid_ctrl_limit, pid_ctrl_limit, false);
    ctrl_y = new PID( Kp_pos, Ki_pos, Kd_pos, -5, 5, -pid_ctrl_limit, pid_ctrl_limit, false);
    ctrl_z = new PID( Kp_height, Ki_height, Kd_height, -5, 5, -pid_ctrl_limit_vert, pid_ctrl_limit_vert, false);
    ctrl_yaw = new PID( Kp_yaw, Ki_yaw, Kd_yaw, -5, 5, -pid_yaw_limit, pid_yaw_limit, false);

    ctrl_x->set_point(target_position[0]);
    ctrl_y->set_point(target_position[1]);
    ctrl_z->set_point(target_position[2]);
    ctrl_yaw->set_point(target_yaw);


    ros::Rate loop_rate(48);

    odometry_sub = nh.subscribe("/guidance/odometry", 1, guidance_odometry_callback);

    ctrl_vel_pub = nh.advertise<geometry_msgs::Vector3>("/control_velocity", 20);
    guidance_bias_pub=nh.advertise<std_msgs::UInt8>("/guidance/bias",20);

    cout << "init success!"<<endl;
    bias_correct_msg.data = 1;

    while(ros::ok())
    {
        guidance_bias_pub.publish(bias_correct_msg);
        cout << "init success!"<<endl;
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
