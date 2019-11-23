#include <string>
#include <math.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

class OdomListener  // Odometry subscriber
{
	public:
		double stamp;
    double x;
    double y;
    double w;
	void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
};
void OdomListener::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	stamp = odom->header.stamp.toSec();
  x = odom->pose.pose.position.x;
  y = odom->pose.pose.position.y;
  w = odom->pose.pose.orientation.w;
}

class GridListener  // Gridmap subscriber
{
	public:
    double stamp;
    double resolution;
    int width;
    int height;
    geometry_msgs::Point pos;
    geometry_msgs::Quaternion dir;
    std::vector<signed char, std::allocator<signed char> > data;
	void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& gridmap);
};
void GridListener::grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& gridmap)
{
  stamp = gridmap->header.stamp.toSec();
  resolution = gridmap->info.resolution;
  width = gridmap->info.width;;
  height = gridmap->info.height;
  pos = gridmap->info.origin.position;
  dir = gridmap->info.origin.orientation;
  data = gridmap->data;
}

// move_base publisher
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n_odom;
  ros::NodeHandle n_gridmap;

  // 'drrobot_X80/odom' topic suscriber
  OdomListener odom;
  ros::Subscriber odom_sub = n_odom.subscribe<nav_msgs::Odometry>("drrobot_X80/odom", 1, &OdomListener::odom_callback,&odom);

  // 'rtabmap/grid_map' topic suscriber
  GridListener gridmap;
  ros::Subscriber gridmap_sub = n_gridmap.subscribe<nav_msgs::OccupancyGrid>("rtabmap/grid_map", 1, &GridListener::grid_callback,&gridmap);
  /*nav_msgs/OccupancyGrid*/

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
  double last_time = ros::Time::now().toSec();

  double goal_x;         // Set goal
  double goal_y;
  double goal_w;

  int value;        // Gridmap vars
  double x;
  double y;

  bool goal_sent = false;

  // Loop rate
  ros::Rate loop_rate(1);	// 1Hz

  while (ros::ok()) {

    ROS_INFO("---------------------------------------------------------------");
    ROS_INFO("Time: %f\tResolution: %f\tOrigin: [%f,%f,%f]", gridmap.stamp, gridmap.resolution, gridmap.pos.x, gridmap.pos.y, gridmap.dir.w);
    ROS_INFO("Height: %d\tWidth: %d\tSize: %d", gridmap.height, gridmap.width, (int)gridmap.data.size());
    
    if (!goal_sent){
      for(int j = 0; j < gridmap.height; j++){
        y = j * gridmap.resolution + gridmap.pos.y;   // y coordinate in meters
        for(int i = 0; i < gridmap.width; i++){
          x = i * gridmap.resolution + gridmap.pos.x; // x coordinate in meters
          value = gridmap.data[j*gridmap.width + i];
          if (value == 100){
            goal_x = x;
            goal_y = y;
            goal_w = 1.0;
            goal.target_pose.pose.position.x = goal_x; // Send goal
            goal.target_pose.pose.position.y = goal_y;
            goal.target_pose.pose.orientation.w = goal_w;

            last_time = ros::Time::now().toSec();

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
            goal_sent = true;
            break;
          }
          //printf("%d ", index);
        }
        //printf("\n");
      }
    }

    //ROS_INFO("time: %f,\tdist: %f,\trot: %f", odom.stamp-last_time, pow( pow(odom.x - goal_x, 2) + pow(odom.y - goal_y, 2), 0.5), fabs(odom.w - goal_w));
    
    if (goal_sent) {
      if( pow( pow(odom.x - goal_x, 2) + pow(odom.y - goal_y, 2), 0.5) < 0.1  // Distance from goal
        && fabs(odom.w - goal_w) < 0.1 )                                 // Difference in orientation
      {
        ac.cancelAllGoals();
        ROS_INFO("\n\tGoal reached");
        goal_sent = false;
        ros::Duration(1.0).sleep();   // Sleep for a second
        break;
      }
      else if ( odom.stamp - last_time > timeout || ac.getState() == actionlib::SimpleClientGoalState::ABORTED)  // Timeout reached
      {
        ac.cancelAllGoals();
        ROS_INFO("\n\tCould not reach goal");
        goal_sent = false;
        ros::Duration(1.0).sleep();   // Sleep for a second
        break;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}