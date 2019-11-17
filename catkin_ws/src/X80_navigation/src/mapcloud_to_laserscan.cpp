#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

class Listener
{
	public:
        //std_msgs/Header header
        std::__cxx11::basic_string<char> frame_id;
        double stamp;
        int height;
        int width;
        //sensor_msgs/PointField[] fields
        int point_step;
        int row_step;
        std::vector<unsigned char, std::allocator<unsigned char> > data;

	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

void Listener::cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    frame_id = cloud_msg->header.frame_id;
    stamp = cloud_msg->header.stamp.toSec();
    height = cloud_msg->height;
    width = cloud_msg->width;
    row_step = cloud_msg->row_step;
    data = cloud_msg->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;

	// 'cloud_map' topic suscriber
	Listener listener;
	ros::Subscriber cloud_map_sub = n.subscribe<sensor_msgs::PointCloud2>("rtabmap/cloud_obstacles", 1, &Listener::cloud_callback,&listener);
	
    int point;

	// 'scan' topic publisher
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);
    sensor_msgs::LaserScan scan;

    unsigned int num_readings = 100;
    double laser_frequency = 40;
    double ranges[num_readings];
    double intensities[num_readings];
    int count = 0;

	// Loop rate
	ros::Rate loop_rate(1);	// 1Hz

	while (ros::ok()) {

        ROS_INFO("Height: %d\tWidth: %d\tRowstep: %d\tSize: %d", listener.height, listener.width, listener.row_step, (int)listener.data.size());
        
        for(int i = 0; i < listener.data.size(); i++)
        {
            point = (int) listener.data[i];
            ROS_INFO("\t%d",point);
        }

        //generate some fake data for our laser scan
        /*for(unsigned int i = 0; i < num_readings; ++i){
            ranges[i] = count;
            intensities[i] = 100 + count;
        }
        ros::Time scan_time = ros::Time::now();
    
        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "camera_link";
        scan.angle_min = -1.57;
        scan.angle_max = 1.57;
        scan.angle_increment = 3.14 / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 100.0;
    
        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);
        for(unsigned int i = 0; i < num_readings; ++i){
            scan.ranges[i] = ranges[i];
            scan.intensities[i] = intensities[i];
        }
   
        scan_pub.publish(scan);
        ++count;*/

        // Wait until next loop rate
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}