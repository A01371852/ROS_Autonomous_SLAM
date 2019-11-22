#include <math.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>

class Listener  // Odometry subscriber
{
	public:
		double stamp;
        double x;
        double y;
        double w;
	void callback(const nav_msgs::Odometry::ConstPtr& odom);
};

void Listener::callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	stamp = odom->header.stamp.toSec();
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    w = odom->pose.pose.orientation.w;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  // 'drrobot_X80/odom' topic suscriber
  Listener listener;
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("drrobot_X80/odom", 1, &Listener::callback,&listener);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 0.5;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  double x = goal.target_pose.pose.position.x;
  double y = goal.target_pose.pose.position.y;
  double w = goal.target_pose.pose.orientation.w;

  ROS_INFO("%f",x);
  ROS_INFO("%f",y);
  ROS_INFO("%f",w);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Loop rate
  ros::Rate loop_rate(1);	// 1Hz

  //ac.waitForResult( ros::Duration(30, 30));
  while (ros::ok()) {
    
    //ROS_INFO("X: %f,\tY: %f", listener.x, listener.y);
    ROS_INFO("dist: %f,\trot: %f", pow( pow(listener.x - x, 2) + pow(listener.y - y, 2), 0.5), fabs(listener.w - w));
    
    if( pow( pow(listener.x - x, 2) + pow(listener.y - y, 2), 0.5) < 0.1 && fabs(listener.w - w) < 0.1){
        ROS_INFO("Hooray, the base moved 1 meter forward");
        ac.cancelAllGoals();
        break;
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}