#include <math.h>
#include <string.h>
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
  Listener odom;
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("drrobot_X80/odom", 1, &Listener::callback,&odom);

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

  //double timeout = 40.0;  // secs
  double timeout = 3600.0;  // secs

  double x = -10.0;         // Set goal
  double y = 0.0;
  double w = -1.0;

  double last_time = ros::Time::now().toSec();

  goal.target_pose.pose.position.x = x; // Send goal
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ros::Duration(0.5).sleep();   // Sleep for half a second

  // Loop rate
  ros::Rate loop_rate(1);	// 1Hz

  while (ros::ok()) {

    //ROS_INFO("time: %f,\tdist: %f,\trot: %f", odom.stamp-last_time, pow( pow(odom.x - x, 2) + pow(odom.y - y, 2), 0.5), fabs(odom.w - w));
    ROS_INFO("result: %d", system(ac.getState().getText().c_str()));
    
    if( pow( pow(odom.x - x, 2) + pow(odom.y - y, 2), 0.5) < 0.1  // Distance from goal
      && fabs(odom.w - w) < 0.1 )                                 // Difference in orientation
    {
      ac.cancelAllGoals();
      ROS_INFO("\n\tGoal reached");
      ros::Duration(1.0).sleep();   // Sleep for a second
      break;
    }
    else if ( odom.stamp - last_time > timeout || ac.getState() == actionlib::SimpleClientGoalState::ABORTED)  // Timeout reached
    {
      ac.cancelAllGoals();
      ROS_INFO("\n\tCould not reach goal");
      ros::Duration(1.0).sleep();   // Sleep for a second
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  last_time = odom.stamp;

  return 0;
}