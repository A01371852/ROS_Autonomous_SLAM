#include <math.h>
#include <vector>
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
  int max_neighbors;
  int count_neighbors;

  bool goal_sent = false;

  // Loop rate
  ros::Rate loop_rate(1);	// 1Hz

  while (ros::ok()) {
    
    // Create a vector containing neighbors
    std::vector<int> neighbors;
    std::vector<int> neighbors_1;

    if (!goal_sent && (int)gridmap.data.size() > 0){
      
      max_neighbors = 0;
      for(int i = 0; i < (int)gridmap.data.size(); i++){
        neighbors_1.push_back(0);
      }

      for (int n = 0; n < 3; n++){  // Iterate gridmap three times
        for(int j = 1; j < gridmap.height-1; j++){
          for(int i = 1; i < gridmap.width-1; i++){
            value = gridmap.data[j*gridmap.width + i];
            if(value == -1){  // If cell is unknown
              count_neighbors = 0;  // Count neighbors
              if(gridmap.data[(j-1)*gridmap.width + i] == -1)
                count_neighbors += 1 + neighbors_1[(j-1)*gridmap.width + i];
              if(gridmap.data[(j+1)*gridmap.width + i] == -1)
                count_neighbors += 1 + neighbors_1[(j+1)*gridmap.width + i];
              if(gridmap.data[j*gridmap.width + (i-1)] == -1)
                count_neighbors += 1 + neighbors_1[j*gridmap.width + (i-1)];
              if(gridmap.data[j*gridmap.width + (i+1)] == -1)
                count_neighbors += 1 + neighbors_1[j*gridmap.width + (i+1)];
              neighbors.push_back(count_neighbors);
              if (count_neighbors > max_neighbors){ // If neighbors are max
                max_neighbors = count_neighbors;
                ROS_INFO("%d", max_neighbors);
                x = i * gridmap.resolution + gridmap.pos.x; // x coordinate in meters
                y = j * gridmap.resolution + gridmap.pos.y; // y coordinate in meters
              }
            }
            else
              neighbors.push_back(0);
          }
        }
        neighbors_1 = neighbors;
        neighbors.clear();
      }
      goal_x = x;
      goal_y = y;
      goal_w = 1.0;
      goal.target_pose.pose.position.x = goal_x; // Send goal
      goal.target_pose.pose.position.y = goal_y;
      goal.target_pose.pose.orientation.w = goal_w;

      last_time = ros::Time::now().toSec();
      ROS_INFO("---------------------------------------------------------------");
      ROS_INFO("Time: %f\tResolution: %f\tOrigin: [%f,%f,%f]", gridmap.stamp, gridmap.resolution, gridmap.pos.x, gridmap.pos.y, gridmap.dir.w);
      ROS_INFO("Height: %d\tWidth: %d\tSize: %d", gridmap.height, gridmap.width, (int)gridmap.data.size());
      ROS_INFO("Sending goal: [%f,%f] : %d", goal_x, goal_y, max_neighbors);
      ac.sendGoal(goal);
      goal_sent = true;
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
      }
      else if ( odom.stamp - last_time > timeout || ac.getState() == actionlib::SimpleClientGoalState::ABORTED)  // Timeout reached
      {
        ac.cancelAllGoals();
        ROS_INFO("\n\tCould not reach goal");
        goal_sent = false;
        ros::Duration(1.0).sleep();   // Sleep for a second
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}