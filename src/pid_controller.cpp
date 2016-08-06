#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <std_msgs/UInt8.h>
#include "PID_lib/pid.h"

#include "demo_flight/pid_ctrl_data.h"
#include "demo_flight/ekf_data.h"

using namespace std;
using namespace ros;
using namespace DJI::onboardSDK;

ros::Publisher ctrl_vel_pub;
ros::Subscriber target_pos_sub;
ros::Subscriber odometry_sub;
ros::Publisher guidance_bias_pub;
PID *ctrl_x;
PID *ctrl_y;
PID *ctrl_z;
PID *ctrl_yaw;

DJIDrone* drone;

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

//void ekf_callback(const demo_flight::ekf_data& ekf);
//void state_judgement(int state_cmd_in,const int state_flight, float* ctrl_data);
//void send_ctrl_to_uav(float* ctrl_data);
void delay(void);

void target_pos_callback(const geometry_msgs::Vector3& target_pos) {

    target_position[0] = target_pos.x;
    target_position[1] = target_pos.y;
    target_position[2] = target_pos.z;
 
}

void guidance_odometry_callback(const nav_msgs::Odometry& guidance_odometry) {

    dt = guidance_odometry.header.stamp.toSec() - first_time;
    first_time = guidance_odometry.header.stamp.toSec();
    

    //pos_data.position_errore_b.x = guidance_odometry.pose.pose.position.x;
    //pos_data.position_errore_b.y = guidance_odometry.pose.pose.position.y;
    //pos_data.position_errore_b.z = guidance_odometry.pose.pose.position.z;

    ctrl_x->set_point(target_position[0]);
    ctrl_y->set_point(target_position[1]);
    ctrl_z->set_point(target_position[2]);
    ctrl_yaw->set_point(0);

    ctrl_data[0] = ctrl_x -> update(guidance_odometry.pose.pose.position.x, dt);
    ctrl_data[1] = ctrl_y -> update(guidance_odometry.pose.pose.position.y, dt);
    //ctrl_data[2] = ctrl_z -> update(-ekf.position_errore_b.z, dt); //there is diffrent yaw in body frame and exact control
    //ctrl_data[3] = ctrl_yaw -> update(ekf.yaw_error, dt);
    ctrl_data[2] = target_position[2]; // position control logic for z-axis
    ctrl_data[3] = 0; //yaw

    if (ctrl_data[0] > pid_ctrl_limit)
        ctrl_data[0] = pid_ctrl_limit;
    if (ctrl_data[0] < -pid_ctrl_limit)
        ctrl_data[0] = -pid_ctrl_limit;
    //ctrl_data[0] = ctrl_data[0] > pid_ctrl_limit ?

    if (ctrl_data[1] > pid_ctrl_limit)
        ctrl_data[1] = pid_ctrl_limit;
    if (ctrl_data[1] < -pid_ctrl_limit)
        ctrl_data[1] = -pid_ctrl_limit;

    /* Publish the output control velocity from PID controller */
    geometry_msgs::Vector3 velocity;

    velocity.x = ctrl_data[0];
    velocity.y = ctrl_data[1];
    velocity.z = ctrl_data[2]; // In fact, the desired position for z-axis
    
    ctrl_vel_pub.publish(velocity);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle nh("~");
    std_msgs::UInt8 bias_correct_msg;
    // ctrl xy
    nh.param("Kp_pos", Kp_pos, 0.8);
    nh.param("Ki_pos", Ki_pos, 0.0);
    nh.param("Kd_pos", Kd_pos, 0.0);

    // ctrl z
    nh.param("Kp_height", Kp_height, 0.4);
    nh.param("Ki_height", Ki_height, 0.0);
    nh.param("Kd_height", Kd_height, 0.0);

    // ctrl yaw
    nh.param("Kp_yaw", Kp_yaw, 0.8);
    nh.param("Ki_yaw", Ki_yaw, 0.0);
    nh.param("Kd_yaw", Kd_yaw, 0.0);

    nh.param("pid_ctrl_limit",pid_ctrl_limit,2.0);
    nh.param("pid_ctrl_limit_vert",pid_ctrl_limit_vert,1.0);
    nh.param("pid_limit_yaw",pid_yaw_limit,25.0);

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

    //ekf_sub = nh.subscribe("/ekf_state", 1, ekf_callback);
    target_pos_sub = nh.subscribe("/target_position", 1, target_pos_callback);
    odometry_sub = nh.subscribe("/guidance/odometry", 1, guidance_odometry_callback);

    ctrl_vel_pub = nh.advertise<geometry_msgs::Vector3>("/control_velocity", 20);
    guidance_bias_pub = nh.advertise<std_msgs::UInt8>("/guidance/bias", 20);

    cout<< "init all and controller start!"<<endl;
    bias_correct_msg.data = 1;

    while(ros::ok())
    {
        guidance_bias_pub.publish(bias_correct_msg);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}


/*
void ekf_callback(const demo_flight::ekf_data& ekf)
{
    //ctrl_x->set_point(target_location[0]);
    //ctrl_y->set_point(target_location[2]);
    //ctrl_z->set_point(target_location[3]);
    //ctrl_yaw->set_point(target_yaw);

    static int state_cmd = 0;
    static int state_flight = 0;
    static float ctrl_data[4] = {0,0,0,0};// v_x v_y v_z, v_yaw

    dt = ekf.delta_t;

    ctrl_data[0] = ctrl_x -> update(ekf.position_errore_b.x, dt);
    ctrl_data[1] = ctrl_y -> update(ekf.position_errore_b.y, dt);
    ctrl_data[2] = ctrl_z -> update(-ekf.position_errore_b.z, dt);//there is diffrent yaw in body frame and exact control
    ctrl_data[3] = ctrl_yaw -> update(ekf.yaw_error, dt);

    if (ctrl_data[0] > pid_ctrl_limit)
        ctrl_data[0] = pid_ctrl_limit;
    if (ctrl_data[0] < -pid_ctrl_limit)
        ctrl_data[0] = -pid_ctrl_limit;

    if (ctrl_data[1] > pid_ctrl_limit)
        ctrl_data[1] = pid_ctrl_limit;
    if (ctrl_data[1] < -pid_ctrl_limit)
        ctrl_data[1] = -pid_ctrl_limit;

    if (ctrl_data[2] > pid_ctrl_limit_vert)
        ctrl_data[2] = pid_ctrl_limit_vert;
    if (ctrl_data[2] < -pid_ctrl_limit_vert)
        ctrl_data[2] = -pid_ctrl_limit_vert;

    if (ctrl_data[3] > pid_yaw_limit)
        ctrl_data[3] = pid_yaw_limit;
    if (ctrl_data[3] < -pid_yaw_limit)
        ctrl_data[3] = -pid_yaw_limit;


    state_judgement(state_cmd, state_flight, ctrl_data);

    cout<<"ctrl_out: "<<endl
       <<ctrl_data<<endl;

}

void state_judgement(int state_cmd_in, const int state_flight, float *ctrl_data)
{
    int state_cmd = state_cmd_in;

    switch (state_cmd)
    {
    case 0:
        if(state_flight == 3)
        {
            float ctrl_data_temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            send_ctrl_to_uav(ctrl_data_temp);
            cout<<"call stand by"<<endl;
        }
        break;
    case 1:
        if(state_flight == 1)
        {
            drone->takeoff();
            cout<<"call take off"<<endl;
        }
        else if(state_flight == 2)
        {
            delay();
            cout<<"delay"<<endl;
        }
        else if(state_flight == 3)
        {
            float ctrl_data_temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            send_ctrl_to_uav(ctrl_data_temp);
            cout<<"call stand by"<<endl;
        }
        break;
    case 2:
        if(state_flight == 3)
        {
            drone ->landing();
            cout<<"call landoing"<<endl;
        }
        else if(state_flight == 4 )
        {
            drone->landing();
            cout<<"call landing and landing"<<endl;
        }
        else if(state_flight == 5)
        {
            drone->drone_disarm();
            cout<<"call dis_arm"<<endl;
        }
        break;
    default:
        if(state_flight == 3)
        {
            send_ctrl_to_uav(ctrl_data);
        }
        break;
    }

}

void send_ctrl_to_uav(float *ctrl_data)
{
    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                             Flight::VerticalLogic::VERTICAL_VELOCITY |
                             Flight::YawLogic::YAW_PALSTANCE |
                             Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                             Flight::SmoothMode::SMOOTH_ENABLE,
                             ctrl_data[0], ctrl_data[1], ctrl_data[2], ctrl_data[3] );// vx, vy, vz, yaw
}
*/
void delay(void)
{
    int count = 100;

    while(count>0)
    {
        count --;
    }
}
