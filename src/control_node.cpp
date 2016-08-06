#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <std_msgs/UInt8.h>
#include "PID_lib/pid.h"

#include <cstdlib>
#include "demo_flight/pid_ctrl_data.h"
//#include <apriltags_opencv_x3/AprilTagDetectionArray.h>
//#include <apriltags_opencv_x3/AprilTagDetection.h>
//#include <Eigen/Dense>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

 #define DEBUG

using namespace DJI::onboardSDK;
using namespace std;
//using namespace Eigen;               //namespace

DJIDrone* drone;

ros::Publisher pid_ctrl;

ros::Subscriber pid_sub;
ros::Subscriber odometry_sub;

float error[3];
float ctrl_data[4] = {0,0,0,0};

float dt = 0.01;
float first_time = 0.0;
float ctrl_x;


//ros::Subscriber odometry_sub;

//void pid_callback(const demo_flight::pid_ctrl_data &pid_data);

void obtain_control_callback(const ros::TimerEvent&)
{
        ROS_INFO("obtain control callback");
        drone->request_sdk_permission_control();
}

void guidance_odometry_callback(const nav_msgs::Odometry& guidance_odometry)
{
    dt = guidance_odometry.header.stamp.toSec() - first_time;
    first_time = guidance_odometry.header.stamp.toSec();


    ctrl_x = guidance_odometry.pose.pose.position.x;
    ROS_INFO("X=%f",ctrl_x);
}

//void pid_callback(const demo_flight::pid_ctrl_data& pid_data)
//{
//    int status = pid_data.status;

//    float vx = pid_data.x;
//    float vy = pid_data.y;
//    float vz = pid_data.z;
//    float yaw = pid_data.yaw;
//    ROS_INFO("vx=%f,vy=%f,vz=%f",vx,vy,vz);

//    error[0] = pid_data.position_error.x;
//    error[1] = pid_data.position_error.y;
//    error[2] = pid_data.position_error.z;

//     drone->takeoff();
//     sleep(8);

////     if (error[0] > 0.15|| error[1] > 0.15|| error[2] > 0.15)
////      {
//          drone->velocity_control(0, vx, vy, vz, yaw );
////      }
////      else
////      {
////          drone->velocity_control(0, 0, 0, 0, 0 );
////      }
//          cout << "pid test" <<endl;
//}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dji_sdk_control");
    ROS_INFO("dji_sdk_control_node");
    ros::NodeHandle nh;
    ros::Timer obtain_control_timer=nh.createTimer(ros::Duration(1.0),obtain_control_callback);  //Duration(1.0)
    drone = new DJIDrone(nh);

    static double vx = 0, vy = 0, vz = 0, yawRate = 0;
//    pid_sub = nh.subscribe("/ctrl_data", 4, pid_callback);
    odometry_sub = nh.subscribe("/guidance/odometry", 1, guidance_odometry_callback);

    ros::Rate rate(50);//控制频率50Hz
    sleep(1);
    while(ros::ok())
    {
        ROS_INFO("OK!");
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
