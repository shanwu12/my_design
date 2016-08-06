#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <apriltags_opencv_x3/AprilTagDetectionArray.h>
#include <Eigen/Dense>
#include <math.h>

 #define DEBUG

using namespace DJI::onboardSDK;
using namespace Eigen;

apriltags_opencv_x3::AprilTagDetectionArray tag_detections;
DJIDrone* drone;
bool pose_update = false;
bool gimbal_update = false; 
bool tag_feedback = false;
bool new_search = true;
bool height_plan = false;
bool landing = false;
double last_tag_time = 0;

void apriltagsCallback(const apriltags_opencv_x3::AprilTagDetectionArray& msg)
{
	if(msg.detections.size() == 1)
	{
		tag_detections = msg;
		pose_update = true;
		gimbal_update = true;
		last_tag_time = ros::Time::now().toSec();
		if(!tag_feedback)
		{
			tag_feedback = true;
			drone->gimbal_speed_control(0, 0, 0);
		}
	}
}

void obtain_control_callback(const ros::TimerEvent&)
{
	//ROS_INFO("obtain control callback");
	drone->request_sdk_permission_control();
}

bool getRelativePose(Vector3d &bodyCoordinate, Vector3d &last_bodyCoordinate)
{
	if(pose_update)
	{
		pose_update = false;
		if(drone->gimbal.pitch>-85)//当pitch角为-90度时，会触发云台的万向锁问题，此时roll与yaw的读数会突变
		{
			last_bodyCoordinate = bodyCoordinate;
			float q0=drone->attitude_quaternion.q0,q1=drone->attitude_quaternion.q1,q2=drone->attitude_quaternion.q2,q3=drone->attitude_quaternion.q3;
			double body_yaw=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
			double alpha = (-90-drone->gimbal.pitch)/180.0*M_PI;//先绕x轴旋转alpha（弧度） 
			//double beta = (drone->gimbal.roll)/180.0*M_PI;//再绕y'轴旋转beta（弧度）
			double beta = 0;  
			double gamma = body_yaw-(90+drone->gimbal.yaw)/180.0*M_PI;////最后绕z''轴旋转gamma（弧度） 
			Vector3d cameraCoordinate(tag_detections.detections[0].pose.position.x, tag_detections.detections[0].pose.position.y, tag_detections.detections[0].pose.position.z);
			Matrix3d cMb;
			double sina=sin(alpha), sinb=sin(beta), sing=sin(gamma), cosa=cos(alpha), cosb=cos(beta), cosg=cos(gamma);
			cMb(0,0) = cosg*cosb;
			cMb(0,1) = sing*cosa + sina*sinb*cosg;
			cMb(0,2) = sing*sina - cosg*sinb*cosa;
			cMb(1,0) = -sing*cosb;
			cMb(1,1) = cosg*cosa - sing*sinb*sina;
			cMb(1,2) = cosg*sina + cosa*sinb*sing;
			cMb(2,0) = sinb;
			cMb(2,1) = -cosb*sina;
			cMb(2,2) = cosb*cosa;
			bodyCoordinate = cMb * cameraCoordinate;
			bodyCoordinate(0) = bodyCoordinate(0);//+0.12
			#ifdef DEBUG
			std::cout<<"x="<<bodyCoordinate(0)<<std::endl;
			std::cout<<"y="<<bodyCoordinate(1)<<std::endl;
			std::cout<<"z="<<bodyCoordinate(2)<<std::endl;
			#endif
			return true;
		}
	}
	return false;
}

void gimbalControl_1d()
{	
	static double pitch_d = 0;
	if(gimbal_update)
	{
		double pitch_e = atan2(-tag_detections.detections[0].pose.position.y, tag_detections.detections[0].pose.position.z) * 180. / M_PI;//角度表示     	
		pitch_d = pitch_e + drone->gimbal.pitch;
		pitch_d = pitch_d > 30 ? 30 : (pitch_d < -85 ? -85 : pitch_d);
		gimbal_update = false;
	}
	if(tag_feedback)
	{
		double pitch_e = (pitch_d - drone->gimbal.pitch) * 10;
		/*bool gimbal_speed_control(int roll_rate = 0, int pitch_rate = 0, int yaw_rate = 0) 输入范围[-1800,1800]*/
		int pitch_rate = static_cast<int>(pitch_e*2);
		pitch_rate = pitch_rate > 1800 ? 1800 : (pitch_rate < -1800 ? -1800 : pitch_rate);
		drone->gimbal_speed_control(0, pitch_rate, 0);
	}
}

void gimbalControl_3d()
{	
	static double pitch_d = 0., yaw_d=0., roll_d = 0.; 
	static bool valid_pose = false;
	if(gimbal_update)
	{

		double pitch_e = atan2(-tag_detections.detections[0].pose.position.y, tag_detections.detections[0].pose.position.z) * 180. / M_PI;//角度表示
		double theta = atan2(tag_detections.detections[0].pose.position.x, tag_detections.detections[0].pose.position.z);//云台绕相机坐标系y轴旋转的弧度
		double delta = drone->gimbal.pitch / 180. * M_PI; //云台相机坐标系y轴与云台偏航轴（z轴）的夹角
		double sint = sin(theta), cost = cos(theta), sind = sin(delta), cosd = cos(delta), sin2d = sind * sind, cos2d = cosd * cosd;
		double yaw_e = atan2(sint * cosd, sin2d + cos2d * cost) * 180. / M_PI;//角度表示
		double roll_e = atan2(sint * sind, cos2d + sin2d * cost) * 180. / M_PI;//角度表示
		/*bool gimbal_angle_control(int roll = 0, int pitch = 0, int yaw = 0, //note: 单位0.1度
		                            int duration = 0, //note: 单位0.1秒
		                            bool absolute_or_incremental = 1, //note: 1表示absolute
                                    bool yaw_cmd_ignore = 0, bool roll_cmd_ignore = 0, bool pitch_cmd_ignore = 0)*/          
		
		pitch_d = pitch_e + drone->gimbal.pitch;
		pitch_d = pitch_d > 30 ? 30 : (pitch_d < -85 ? -85 : pitch_d);
		roll_d = roll_e + drone->gimbal.roll;
		roll_d = roll_d > 35 ? 35 : (roll_d < -35 ? -35 : roll_d);
		yaw_d = yaw_e + drone->gimbal.yaw;
		yaw_d = yaw_d > 180 ? (180 - yaw_d) : (yaw_d < -180 ? (360 + yaw_d) : yaw_d);
		#ifdef DEBUG
		std::cout<<"roll="<<roll_e<<std::endl;
		std::cout<<"pitch="<<pitch_e<<std::endl;
		std::cout<<"yaw="<<yaw_e<<std::endl;
		#endif
		valid_pose = true;
		gimbal_update = false;
	}
	if(valid_pose)
	{
		double pitch_e = (pitch_d - drone->gimbal.pitch) * 10;
		double roll_e = (roll_d - drone->gimbal.roll) * 10;
		double yaw_e = (yaw_d - drone->gimbal.yaw) * 10;
		/*bool gimbal_speed_control(int roll_rate = 0, int pitch_rate = 0, int yaw_rate = 0) 输入范围[-1800,1800]*/
		int roll_rate = static_cast<int>(roll_e*3);
		roll_rate = roll_rate > 1800 ? 1800 : (roll_rate < -1800 ? -1800 : roll_rate);
		int pitch_rate = static_cast<int>(pitch_e*3);
		pitch_rate = pitch_rate > 1800 ? 1800 : (pitch_rate < -1800 ? -1800 : pitch_rate);
		int yaw_rate = static_cast<int>(yaw_e*3);
		yaw_rate = yaw_rate > 1800 ? 1800 : (yaw_rate < -1800 ? -1800 : yaw_rate);
		drone->gimbal_speed_control(roll_rate, pitch_rate, yaw_rate);
	}
}

void gimbal_search_tag()
{
	static int tag_search_stage;
	static float desire_gimbal_yaw;
	static int yaw_direction=0;
	if(new_search)
	{
		desire_gimbal_yaw = drone->gimbal.yaw > 0 ? drone->gimbal.yaw -180 : drone->gimbal.yaw + 180;
		yaw_direction = drone->gimbal.yaw > 0 ? -1 : 1;
		tag_search_stage = 0x00;
		new_search = false;
	}
	switch(tag_search_stage)
	{
		case 0x00:
	    {
	    	if(drone->gimbal.pitch > -80)
	    	{
	    		drone->gimbal_speed_control(0, -200, 0);
	    	}
	    	else
	    	{
	    		drone->gimbal_speed_control(0, 0, 0);
	    		tag_search_stage = 0x01;
	    	}
	    	break;
	    }
	    case 0x01:
	    {
	    	if(drone->gimbal.pitch < -18)
	    	{
	    		drone->gimbal_speed_control(0, 200, 0);
	    	}
	    	else
	    	{
	    		drone->gimbal_speed_control(0, 0, 0);
	    		tag_search_stage = 0x02;
	    	}
	    	break;
	    }
	    case 0x02:
	    {
	    	if(yaw_direction==1 && drone->gimbal.yaw < desire_gimbal_yaw)
	    	{
	    		drone->gimbal_speed_control(0, 0,200);
	    	}
	    	else if(yaw_direction==-1 && drone->gimbal.yaw > desire_gimbal_yaw )
	    	{
	    		drone->gimbal_speed_control(0, 0, -200);
	    	}
	    	else
	    	{
	    		tag_search_stage = 0x03;
	    		drone->gimbal_speed_control(0, 0, 0);
	    	}
	    	break;
	    }
	    case 0x03:
	    {
	    	if(drone->gimbal.pitch < -18)
	    	{
	    		drone->gimbal_speed_control(0, 200, 0);
	    	}
	    	else
	    	{
	    		drone->gimbal_speed_control(0, 0, 0);
	    	}
	    	break;
	    }
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dji_sdk_control");
    ROS_INFO("dji_sdk_control_node");
    ros::NodeHandle nh;
    ros::Subscriber apriltags_sub = nh.subscribe("/tag_detections", 1, apriltagsCallback);
    ros::Timer obtain_control_timer=nh.createTimer(ros::Duration(1.0),obtain_control_callback);//unknown
    drone = new DJIDrone(nh);
    ros::Rate rate(50);//控制频率50Hz
    Vector3d bodyCoor(0, 0, 0), last_bodyCoor(0,0,0);
    static double vx = 0, vy = 0, vz = 0, yawRate = 0;
    static double last_z_err = 0;
    double alpha = -1;//alpha表示飞机与标志物中心连线与地面的夹角
    sleep(1);
    while(ros::ok())
    {
		ros::spinOnce();
		if(drone->rc_channels.gear > -5000)//gearDown
		{
			if(!tag_feedback && !landing)
			{
				gimbal_search_tag();
			}
			else
			{				
				//gimbalControl_3d();
				gimbalControl_1d();			
				if(getRelativePose(bodyCoor, last_bodyCoor))//获得AprilTag在机体坐标系下的位置
				{
					if(!height_plan)
					{
						Vector3d temp = bodyCoor - last_bodyCoor;
						double error = temp(0)*temp(0) + temp(1)*temp(1) + temp(2)*temp(2);
						if(error<0.6)
						{
							double hor_dist = sqrt(bodyCoor(0)*bodyCoor(0) + bodyCoor(1)*bodyCoor(1));
							alpha = atan((bodyCoor(2)-3)/hor_dist);						
							height_plan = true;
						}
						#ifdef DEBUG
						std::cout<<"alpha="<<alpha<<std::endl;
						#endif
					}
					double hor_dist = sqrt(bodyCoor(0)*bodyCoor(0) + bodyCoor(1)*bodyCoor(1));
					float kpx = fabs(bodyCoor(0)) > 4.8 ? 0.5 : 0.13;
					float kpy = fabs(bodyCoor(1)) > 4.8 ? 0.5 : 0.13;
					vx = bodyCoor(0) * kpx + (bodyCoor(0) - last_bodyCoor(0)) * 0.1;
					vy = bodyCoor(1) * kpy + (bodyCoor(1) - last_bodyCoor(1)) * 0.1;//PD控制
					if(!landing)
					{
						if(alpha>0)
						{
							double z_d = 3 + hor_dist * tan(alpha);
							double z_err = z_d - bodyCoor(2);
							if(z_err < 0)
								vz = z_err * 1 + (z_err - last_z_err) * 0.1; 
							last_z_err = z_err;
							#ifdef DEBUG
							std::cout<<"z_d="<<z_d<<std::endl;
							#endif
						}
						else
						{
							vz = 0;
						}
					}
							
					vx = vx > 1.5 ? 1.5 : (vx < -1.5? -1.5 : vx);
					vy = vy > 1.5 ? 1.5 : (vy < -1.5? -1.5 : vy);//输出限幅
					vz = vz > 1 ? 1 : (vz < -1? -1 : vz);
					if(fabs(bodyCoor(0)) < 0.1 && fabs(bodyCoor(1)) < 0.1)
					{
						vz = -0.4;
						landing = true;
					}
					#ifdef DEBUG
					std::cout<<"vx="<<vx<<std::endl;
					std::cout<<"vy="<<vy<<std::endl;
					std::cout<<"vz="<<vz<<std::endl;
					#endif
				}	
				if( (ros::Time::now().toSec() - last_tag_time) > 0.4 )
				{
					tag_feedback = false;
					new_search = true;
					vx = vy = 0;
				}
				drone->velocity_control(0, vx, vy, vz, yawRate);//velocity_control的第一个参数frame:0表示机体坐标系，1表示世界坐标系
				rate.sleep();
			}
		}
		else if(drone->rc_channels.gear < -5000)//gearUp
		{
	    	drone->velocity_control(0, 0, 0, 0, 0);//velocity_control的第一个参数frame:0表示机体坐标系，1表示世界坐标系
	    	vx = 0; vy = 0; vz = 0;
	    	landing = false;
	    	height_plan = false;
	    	alpha = -1;
	    	last_z_err = 0;
	    	bodyCoor(0) = 0;
	    	bodyCoor(1) = 0;
	    	bodyCoor(2) = 0;
		}
    }
    return 0;
}
