#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <string>

ros::Publisher pub;
laser_geometry::LaserProjection projector_;

void scan_to_XY (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  // Create a container for the data.
  sensor_msgs::PointCloud cloud;

  //convert scan data into point cloud
  projector_.projectLaser(*scan_in, cloud);

 //publish point cloud
  pub.publish(cloud);  

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "scan_to_xy");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Converting /scan to XY point cloud");

  // Create a ROS subscriber for the input laser scan
  std::string scan_topic = "scan", output = "scan_to_xy_out";
  
  nh.param("scan_topic", scan_topic, scan_topic);
  nh.param("pcl_output", output, output);

  ros::Subscriber scanSub;
  scanSub = nh.subscribe(scan_topic, 1, scan_to_XY);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud>(output, 1);

  // Spin
  ros::spin ();
}
