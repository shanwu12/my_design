#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <std_msgs/UInt8.h>
#include "PID_lib/pid.h"

#include <cstdlib>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "dji_sdk_control/pid_ctrl_data.h"

 #define DEBUG

using namespace DJI::onboardSDK;
using namespace std;

enum uav_status
{
    uav_standby,
    uav_search_tag,
    tag_guide,
};

DJIDrone* drone;

//ros::Publisher pid_ctrl;

ros::Subscriber ctrl_vel_sub;
//ros::Subscriber odometry_sub;
float ctrl_x;
float error[3];


//ros::Subscriber odometry_sub;

//void pid_callback(const dji_sdk_control::pid_ctrl_data &pid_data);

void obtain_control_callback(const ros::TimerEvent&)
{
        ROS_INFO("obtain control callback");
        drone->request_sdk_permission_control();
}

//void pid_callback(const dji_sdk_control::pid_ctrl_data& pid_data)
//{
//    int status = pid_data.status;

//    float vx = pid_data.x;
//    float vy = pid_data.y;
//    float vz = pid_data.z;
//    float yaw = pid_data.yaw;

//    error[0] = pid_data.position_error.x;
//    error[1] = pid_data.position_error.y;
//    error[2] = pid_data.position_error.z;
//    if(status == 1)
//    {
//        drone ->takeoff();
//        sleep(2);
//        if(error[0] > 0.15) || error[0] > 0.15) ||error[0] > 0.15)
//        {
//            drone ->velocity_control(0,vx,vy,yz,yaw);
//        }
//    }
//    int status = pid_data.x;
//}

void ctrl_vel_callback(const geometry_msgs::Vector3& ctrl_vel)
{
    float vx = ctrl_vel.x;
    float vy = ctrl_vel.y;
    float vz = ctrl_vel.z;
    float yaw = 0;
    cout<<"ctrl_vel_callback"<<endl;
    cout<<"vz=%f"<<vz<<endl;

   // drone->velocity_control(0, vx, vy, vz, yaw );
    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            vx, vy, vz, yaw);
   usleep(20000);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dji_sdk_control");
    ROS_INFO("dji_sdk_control_node");
    ros::NodeHandle nh;
    ros::Timer obtain_control_timer=nh.createTimer(ros::Duration(1.0),obtain_control_callback);  //Duration(1.0)
    drone = new DJIDrone(nh);

    ctrl_vel_sub = nh.subscribe("/control_velocity",1,ctrl_vel_callback);

        drone->request_sdk_permission_control();
        drone -> takeoff();
        sleep(8);

    ros::Rate rate(50);//控制频率50Hz
    sleep(1);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
