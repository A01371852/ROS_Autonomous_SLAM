#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <drrobot_X80_player/MotorInfo.h>
#include <drrobot_X80_player/MotorInfoArray.h>

class Listener
{
	public:
		int L_pos, L_dir;
		int R_pos, R_dir;
		//double ros::Time L_tim, R_tim;

	void callback(const drrobot_X80_player::MotorInfoArray::ConstPtr& drrobot_motor);
};

void Listener::callback(const drrobot_X80_player::MotorInfoArray::ConstPtr& drrobot_motor)
{
	//L_tim = drrobot_motor->motorInfos[0].header.stamp;
	L_pos = drrobot_motor->motorInfos[0].encoder_pos;
	L_dir = drrobot_motor->motorInfos[0].encoder_dir;
	if(L_dir > 1)	L_dir = -1;

	//R_tim = drrobot_motor->motorInfos[1].header.stamp;
	R_pos = drrobot_motor->motorInfos[1].encoder_pos;
	R_dir = drrobot_motor->motorInfos[1].encoder_dir;
	if(R_dir > 1)	R_dir = -1;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	Listener listener;
	ros::Subscriber motor_info_sub;
	motor_info_sub = n.subscribe("drrobot_motor",1000,&Listener::callback,&listener);
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

	
	// initial position
	double x = 0.0; 
	double y = 0.0;
	double th = 0;

	// wheels angular displacement (rad) and velocity (rad/s)
	double oL, wL, tL_1 = 0.0;
	double oR, wR, tR_1= 0.0;
	int L_pos_1 = listener.L_pos;
	int R_pos_1 = listener.R_pos;
	// wheels linear displacement (m) and velocity (m/s)
	double dL, vL = 0.0;
	double dR, vR = 0.0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	const double degree		= M_PI/180;
	const double pi_2		= 2*M_PI;
	const double wheelRadius= 0.0855;
    const double wheelDis	= 0.287;
	const int encoderCnt	= 756;		// Encoder ticks per rev
	const int max_Cnt		= 32768;	// Max ticks count (32767 = -1)

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	while (ros::ok()) {

		// 1 -> forward; -1 -> backwards
		ROS_INFO("\t L_pos: %d | L_dir: %d", listener.L_pos, listener.L_dir);
		ROS_INFO("\t R_pos: %d | R_dir: %d", listener.R_pos, listener.R_dir);
		//ROS_INFO("\n");
		
		// Left wheel
		/*oL = listener.L_dir * (((listener.L_pos - L_pos_1) * (-listener.L_dir)) % max_Cnt) * pi_2 / encoderCnt;
		dL += oL * wheelRadius;*/
		//wL = oL / (listener.L_tim - tL_1);
		oL -= listener.L_dir * (((listener.L_pos - L_pos_1) * (listener.L_dir)) % max_Cnt);
		L_pos_1 = listener.L_pos;
		//tL_1 = listener.L_tim;
		
		// Right wheel
		/*oR = listener.R_dir * (((listener.R_pos - R_pos_1) * (-listener.R_dir)) % max_Cnt) * pi_2 / encoderCnt;
		dR += oR * wheelRadius;*/
		//wR = oR / (listener.R_tim - tR_1);
		oR -= listener.L_dir * (((listener.R_pos - R_pos_1) * (listener.R_dir)) % max_Cnt);
		R_pos_1 = listener.R_pos;
		//tR_1 = listener.R_tim;
		
		/*ROS_INFO("dL: %.1f | oL: %.1f", dL, oL);
		ROS_INFO("dR: %.1f | oR: %.1f", dR, oR);
		ROS_INFO("\n-------------------\n");*/
		
/*
		double vx= -(listener.vxo);
		double vth= -((1.5)*(listener.vtho));
		current_time = ros::Time::now(); 
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		odom_trans.header.stamp = current_time; 
		odom_trans.transform.translation.x = x; 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vth;

		last_time = current_time;

		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);
*/
		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
