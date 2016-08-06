#include <iostream>
#include <stdio.h>
#include <cstdlib>

#include <ros/ros.h>

#include <dji_sdk/dji_drone.h>

//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>

#include "demo_flight/pid_ctrl_data.h"
//#include "pid_ctrl_data.h"

using namespace std;
using namespace DJI::onboardSDK;

ros::Subscriber pid_sub;

DJIDrone* drone;

float error[3];

void delay();
void pid_callback(const demo_flight::pid_ctrl_data &pid_data);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sdk_client1");
    ros::NodeHandle nh;

    drone = new DJIDrone(nh);

    ROS_INFO("sdk_service_client_test");

    pid_sub = nh.subscribe("/ctrl_data", 4, pid_callback);


    ros::spin();

    return 0;
}

void pid_callback(const demo_flight::pid_ctrl_data& pid_data)
{
    int status = pid_data.status;

    float vx = pid_data.x;
    float vy = pid_data.y;
    float vz = pid_data.z;
    float yaw = pid_data.yaw;

    error[0] = pid_data.position_error.x;
    error[1] = pid_data.position_error.y;
    error[2] = pid_data.position_error.z;

    if(status == 1)//pitch up
    {
        drone->takeoff();
        delay();

        if (error[0] > 0.15
                || error[1] > 0.15
                || error[2] > 0.15)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                     Flight::VerticalLogic::VERTICAL_VELOCITY |
                                     Flight::YawLogic::YAW_PALSTANCE |
                                     Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                     Flight::SmoothMode::SMOOTH_ENABLE,
                                     vx, vy, vz, yaw );
        }
        else
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                     Flight::VerticalLogic::VERTICAL_VELOCITY |
                                     Flight::YawLogic::YAW_PALSTANCE |
                                     Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                     Flight::SmoothMode::SMOOTH_ENABLE,
                                     0, 0, 0, 0 );
        }
    }
    else if(status==2)// pitch down
    {
        drone->landing();
        delay();
    }
    else
    {

        if (error[0] > 0.15
                || error[1] > 0.15
                || error[2] > 0.15)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                     Flight::VerticalLogic::VERTICAL_VELOCITY |
                                     Flight::YawLogic::YAW_PALSTANCE |
                                     Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                     Flight::SmoothMode::SMOOTH_ENABLE,
                                     vx, vy, vz, yaw );
        }
        else
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                     Flight::VerticalLogic::VERTICAL_VELOCITY |
                                     Flight::YawLogic::YAW_PALSTANCE |
                                     Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                     Flight::SmoothMode::SMOOTH_ENABLE,
                                     0, 0, 0, 0 );
        }
    }


}

void delay()
{
    for(int i=0;i<10;i++)
        ;
}
