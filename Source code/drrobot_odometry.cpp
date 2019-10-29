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
		int L_pos;
		int L_dir;
		int R_pos;
		int R_dir;
		double L_tim;
		double R_tim;

	void callback(const drrobot_X80_player::MotorInfoArray::ConstPtr& drrobot_motor);
};

void Listener::callback(const drrobot_X80_player::MotorInfoArray::ConstPtr& drrobot_motor)
{
	L_tim = drrobot_motor->motorInfos[0].header.stamp.toSec();
	L_pos = drrobot_motor->motorInfos[0].encoder_pos;
	L_dir = -drrobot_motor->motorInfos[0].encoder_dir;
	if(L_dir > 1)	L_dir = -1;		// 1 -> forward; -1 -> backwards

	R_tim = drrobot_motor->motorInfos[1].header.stamp.toSec();
	R_pos = drrobot_motor->motorInfos[1].encoder_pos;
	R_dir = drrobot_motor->motorInfos[1].encoder_dir;
	if(R_dir > 1)	R_dir = -1;		// 1 -> forward; -1 -> backwards
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;

	// 'drrobot_motor' topic suscriber
	Listener listener;
	ros::Subscriber motor_info_sub;
	motor_info_sub = n.subscribe("drrobot_motor",1000,&Listener::callback,&listener);
	
	// 'odom' topic publisher
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

	// Message declarations
	nav_msgs::Odometry odom;
	geometry_msgs::Quaternion odom_quat;
	// Transform declarations
	tf::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	// Wheels angular displacement (ticks)
	int oL = 0;
	int oR = 0;
	int L_pos_1 = listener.L_pos;
	int R_pos_1 = listener.R_pos;
	// Wheels linear displacement (m)
	double dL = 0.0;
	double dR = 0.0;
	double sL = 0.0;
	double sR = 0.0;
	// Wheels angular velocity (rad/s)
	double vL = 0.0;
	double vR = 0.0;
	double L_tim_1 = listener.L_tim;
	double R_tim_1 = listener.R_tim;
	// Delay counter
	int delayCount = 0;
	// Robot linear (m/s) and angular (rad/s) displacements
	double d = 0.0;
	double o = 0.0;
	// Robot linear (m/s) and angular (rad/s) velocities
	double v = 0.0;
	double w = 0.0;
	// Robot initial position (m) and direction (rad)
	double x = 0.0; 
	double y = 0.0;
	double th = 0.0;
	// Robot initial velocities (m/s)
	double vx = 0.0;
	double vy = 0.0;

	// DrRobot X80 constants
	const double wheelRadius= 0.0855;
    const double wheelDis	= 0.287;
	const int encoderCnt	= 756;		// Encoder ticks per rev
	const int max_Cnt		= 32768;	// Max ticks count (32767 = -1)

	// Loop rate
	ros::Rate loop_rate(20);	// 20Hz

	while (ros::ok()) {

		if (delayCount < 10)	// Wait for messages to stabilize
		{
			delayCount++;
			L_pos_1 = listener.L_pos;
			R_pos_1 = listener.R_pos;
			L_tim_1 = listener.L_tim;
			R_tim_1 = listener.R_tim;
			oL = 0;
			oR = 0;
			//ROS_INFO("%d", listener.L_pos-L_pos_1);
		}
		else 
		{		
			/*ROS_INFO("\t L_pos: %d | L_dir: %d", listener.L_pos, listener.L_dir);
			ROS_INFO("\t R_pos: %d | R_dir: %d", listener.R_pos, listener.R_dir);
			ROS_INFO("\n");*/
			
			// Left wheel
			if(abs(listener.L_pos - L_pos_1) < 100)
				oL = -listener.L_dir * (((listener.L_pos - L_pos_1) * (listener.L_dir)) % max_Cnt);
			else
			{
				oL = 0.0;
				ROS_INFO("Encoder error!");
			}
			dL = oL * wheelRadius * 2.0 * M_PI / encoderCnt;
			//sL += dL;
			if (listener.L_tim != L_tim_1)
				vL = dL / (listener.L_tim - L_tim_1);
			L_pos_1 = listener.L_pos;
			L_tim_1 = listener.L_tim;
			
			// Right wheel
			if(abs(listener.R_pos - R_pos_1) < 100)
				oR = listener.R_dir * (((listener.R_pos - R_pos_1) * (listener.R_dir)) % max_Cnt);
			else
			{
				oR = 0.0;
				ROS_INFO("Encoder error!");
			}
			dR = oR * wheelRadius * 2.0 * M_PI / encoderCnt;
			//sR += dR;
			if (listener.R_tim != R_tim_1)
			{
				vR = dR / (listener.R_tim - R_tim_1);
				o = (dR - dL) / (2.0 * wheelDis);
			}
			R_pos_1 = listener.R_pos;
			R_tim_1 = listener.R_tim;
			
			/*ROS_INFO("\n-------------------\n");
			ROS_INFO("time: %.2f", listener.L_tim);
			ROS_INFO("sL: %.2f | vL: %.2f | dirL: %d", sL, vL, listener.L_dir);
			ROS_INFO("sR: %.2f | vR: %.2f | dirR: %d", sR, vR, listener.R_dir);*/

			// Linear and angular displacement of differential drive robot
			d = (dR + dL) / 2.0;
			//o = (dR - dL) / (2.0 * wheelDis);

			// Linear and angular velocities of differential drive robot
			v = (vR + vL) / 2.0;
			w = (vR - vL) / (2.0 * wheelDis);

			// Absolute position and direction of robot
			x += d * cos(th + (o / 2.0));
			y += d * sin(th + (o / 2.0));
			th += o;

			// Axis velocities of robot
			vx = v * cos(th);
			vy = v * sin(th);

			// Rotation quaternion
			odom_quat = tf::createQuaternionMsgFromYaw(th);

			// Update transform
			odom_trans.header.stamp = ros::Time::now(); 
			odom_trans.transform.translation.x = x; 
			odom_trans.transform.translation.y = y; 
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			// Filling the odometry
			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_footprint";

			// Position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			// Velocity
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.linear.z = 0.0;
			odom.twist.twist.angular.x = 0.0; // roll
			odom.twist.twist.angular.y = 0.0; // pitch
			odom.twist.twist.angular.z = w;	  // yaw

			// Publishing tf and odometry
			broadcaster.sendTransform(odom_trans);
			odom_pub.publish(odom);
		}

		// Wait until next loop rate
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}