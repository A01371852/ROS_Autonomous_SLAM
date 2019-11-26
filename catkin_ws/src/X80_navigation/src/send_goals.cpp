#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

class OdomListener  // Odometry subscriber
{
	public:
		double stamp;
		double x;
		double y;
		geometry_msgs::Quaternion orientation;
	void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
};
void OdomListener::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	stamp = odom->header.stamp.toSec();
	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;
	orientation = odom->pose.pose.orientation;
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
	ros::NodeHandle n_grid;
	ros::NodeHandle n_vel;

	// 'drrobot_X80/odom' topic suscriber
	OdomListener odom;
	ros::Subscriber odom_sub = n_odom.subscribe<nav_msgs::Odometry>("/drrobot_X80/odom", 1, &OdomListener::odom_callback,&odom);

	// 'rtabmap/grid_map' topic suscriber
	GridListener gridmap;
	ros::Subscriber gridmap_sub = n_grid.subscribe<nav_msgs::OccupancyGrid>("/rtabmap/grid_map", 1, &GridListener::grid_callback,&gridmap);
	
	// 'cmd_vel' publisher
	geometry_msgs::Twist cmd_vel;
	ros::Publisher cmd_vel_pub = n_vel.advertise<geometry_msgs::Twist>("/drrobot_X80/cmd_vel", 1);

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	// move_base publisher
	move_base_msgs::MoveBaseGoal goal;

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "map";

	//double timeout = 60.0;  // secs
	double timeout = 150.0;  // secs
	double last_time = ros::Time::now().toSec();

	double goal_x;         // Set goal
	double goal_y;
	double goal_w;

	int value;        // Gridmap vars
	double x;
	double y;
	double x_temp;
	double y_temp;
	double dist_temp;
	double min_distance;
	int max_neighbors;
	int count_neighbors;
	std::vector<int> neighbors;	// Vector containing neighbors
	std::vector<int> neighbors_1;
	int unknown;

	double w;	// cmd_vel

	int state = 0;	// Current state

	// Loop rate
	ros::Rate loop_rate(1);	// 1Hz

	while (ros::ok()) {
		
		switch (state)	// FSM
		{
		// ------------------------------------------- Find closest unknown area -------------------------------
		case 0:
			
			if ( (int)gridmap.data.size() > 0 ){	// If map is not empty
				
				// Fill previous values vector with zeros
				for(int i = 0; i < (int)gridmap.data.size(); i++)
					neighbors_1.push_back(0);
				
				// Iterate gridmap twice
				for (int n = 0; n < 2; n++){
					max_neighbors = 0;	// Initialize selection metrics
					min_distance = 50;
					for (int j = 0; j < gridmap.height; j++){
						for (int i = 0; i < gridmap.width; i++){
							value = gridmap.data[j*gridmap.width + i];	// Read cell
							// If cell is known || belongs to first and last rows or columns
							if ( value != -1 || i == 0 || i == gridmap.width-1 || j == 0 || j == gridmap.height-1)
								neighbors.push_back(0);
							else
							{
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
								if (count_neighbors >= max_neighbors){ // If neighbors are max
									max_neighbors = count_neighbors;
									x_temp = i * gridmap.resolution + gridmap.pos.x; // x coordinate in meters
									y_temp = j * gridmap.resolution + gridmap.pos.y; // y coordinate in meters
									dist_temp = pow( pow(odom.x - x_temp, 2) + pow(odom.y - y_temp, 2), 0.5);
									if (dist_temp < min_distance && dist_temp > 0.20){  // If it is the closest unknown area
										min_distance = dist_temp;
										x = x_temp;	// Set new goal
										y = y_temp;
									}
								}
							}
						}
					}
					neighbors_1 = neighbors;	// Ready for new iteration
					neighbors.clear();
				}
				goal_x = x;	// Set goal
				goal_y = y;
				goal_w = atan2(y, x);
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = goal_x; // Send goal
				goal.target_pose.pose.position.y = goal_y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_w);
				ac.sendGoal(goal);

				last_time = ros::Time::now().toSec();	// Start timeout counter

				ROS_INFO("\nSTATE 0: ---------------------------------------------");
				ROS_INFO("Time: %.2f  Origin: [%.2f,%.2f,%.2f]", gridmap.stamp, gridmap.pos.x, gridmap.pos.y, gridmap.dir.w);
				ROS_INFO("Height: %d  Width: %d  Size: %d", gridmap.height, gridmap.width, (int)gridmap.data.size());
				ROS_INFO("Sending goal: [%.2f,%.2f]", goal_x, goal_y);
				ROS_INFO("Value: %d Distance: %.2f", max_neighbors, min_distance);

				state = 1;	// Goal is sent
			}
			else
				state = 0;
			break;
		
		// --------------------------------------- Monitor state of current goal -------------------------------
		case 1:
			
			//ROS_INFO("time: %.2f,\tdist: %.2f,\trot: %.2f", odom.stamp-last_time, pow( pow(odom.x - goal_x, 2) + pow(odom.y - goal_y, 2), 0.5), fabs(tf::getYaw(odom.orientation) - goal_w));
			if( pow( pow(odom.x - goal_x, 2) + pow(odom.y - goal_y, 2), 0.5) < 0.16  // Distance from goal
				&& fabs( sin(tf::getYaw(odom.orientation)) - sin(goal_w)) < 0.1      // Difference in orientation
				&& fabs( cos(tf::getYaw(odom.orientation)) - cos(goal_w)) < 0.1 )
			{
				ac.cancelGoal();	// Clear goals
				ROS_INFO("\nSTATE 1: ---------------------------------------------");
				ROS_INFO("Goal reached");
				w = tf::getYaw(odom.orientation);	// Current orientation
				last_time = ros::Time::now().toSec();	// Start countdown
				state = 2;	// Goal succeded
			}
			else if ( odom.stamp - last_time > timeout || ac.getState() == actionlib::SimpleClientGoalState::ABORTED)  // Goal aborted
			{
				ac.cancelGoal();	// Clear goals
				ROS_INFO("\nSTATE 1: ---------------------------------------------");
				ROS_INFO("Goal is unreachable");

				if( pow( pow(odom.x, 2) + pow(odom.y, 2), 0.5) < 0.15	// Since the robot did not move from the origin
				&& fabs(tf::getYaw(odom.orientation)) < 0.2	)			// there are no more areas of interest
				{
					state = 5;	// Finish execution
				}
				else	// There might be more areas of interest left to explore
				{
					w = tf::getYaw(odom.orientation);	// Current orientation
					last_time = ros::Time::now().toSec();	// Start countdown
					state = 2;	// Goal was aborted
				}
			}
			else
				state = 1;
			break;

		// ----------------------------------------------- Perform full rotation -------------------------------
		case 2:
			
			if (odom.stamp - last_time > 3.0 && fabs(tf::getYaw(odom.orientation) - w) < 0.12)	// If rotation was completed
			{
				cmd_vel.linear.x = 0.0;	// Stop rotation
            	cmd_vel.angular.z = 0.0;
				cmd_vel_pub.publish(cmd_vel);
				state = 3;
			}
			else
			{
				ROS_INFO("\nSTATE 2: ---------------------------------------------");
				ROS_INFO("Performing full rotation");
				cmd_vel.linear.x = 0.0;	// Send rotation command
				cmd_vel.angular.z = 0.11;
				cmd_vel_pub.publish(cmd_vel);
				state = 2;
			}
			break;
		
		// ------------------------------------------------- Send goal to origin -------------------------------
		case 3:
			if (odom.stamp - last_time > 3.0 && fabs(tf::getYaw(odom.orientation) - w) < 0.12)	// If rotation was completed
			{
				ac.cancelGoal();	// Clear goals
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = 0.0; // Send goal to origin
				goal.target_pose.pose.position.y = 0.0;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				ac.sendGoal(goal);
				
				ROS_INFO("\nSTATE 3: ---------------------------------------------");
				ROS_INFO("Rotation completed\nGoing home");
				last_time = ros::Time::now().toSec();
				state = 4;	// Go to origin
			}
			else
				state = 3;
			break;
		
		// -------------------------------------------- Wait to return to origin -------------------------------
		case 4:
			
			// If the base is in the origin or the plan was aborted
			if( pow( pow(odom.x, 2) + pow(odom.y, 2), 0.5) < 0.15
				&& fabs(tf::getYaw(odom.orientation)) < 0.2
				|| ac.getState() == actionlib::SimpleClientGoalState::ABORTED
				|| odom.stamp - last_time > timeout)
			{
				ROS_INFO("\nSTATE 4: ---------------------------------------------");
				ROS_INFO("Back home\nFinding a new goal");
				ac.cancelGoal();	// Clear all goals
				state = 0;	// Find new goal
			}
			else
				state = 4;
			break;

		// ----------------------------------------------------- Map is complete  -------------------------------
		case 5:
			
			for(int i = 0; i < (int)gridmap.data.size(); i++)	// Count unknown grids
				if (gridmap.data[i] == -1) unknown++;
			ROS_INFO("\nSTATE 5: ---------------------------------------------");
			ROS_INFO("%.2f percent of the map is known", (double)unknown/(double)gridmap.data.size());
			ROS_INFO("THE MAP IS COMPLETE!");
			state = 5;
			return 0;
			break;

		default:
			ROS_INFO("\nSomething is wrong!");
			ac.cancelGoal();	// Clear all goals
			state = 0;
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}