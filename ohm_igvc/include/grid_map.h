#ifndef _GRID_MAP_H_
#define _GRID_MAP_H_

// ros includes

#include <ros/ros.h> 
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>

// opencv includes

#include <opencv2/opencv.hpp>

// stl includes

#include <string>
#include <cmath>
#include <queue>

class grid_map {
	public:
		grid_map(int width, int height, double resolution, int threshold, geometry_msgs::Point raster_reference); // 
		void laser_scan_update(const sensor_msgs::LaserScan::ConstPtr &scan); //  
		void point_cloud_update(const sensor_msgs::PointCloud::ConstPtr &points); // 
		void odometry_update(const nav_msgs::Odometry::ConstPtr &odom); // 
		void display_map(const ros::TimerEvent &e);

	private:
		bool successors_callback(ohm_igvc::successors::Request &rq, ohm_igvc::successors::Response &rp); // 
		bool cell_to_real_callback(ohm_igvc::cell_to_real::Request &rq, ohm_igvc::cell_to_real::Response &rp); //
		bool real_to_cell_callback(ohm_igvc::real_to_cell::Request &rq, ohm_igvc::real_to_cell::Response &rp); //
		bool robot_position_callback(ohm_igvc::robot_position::Request &rq, ohm_igvc::robot_position::Response &rp); //
		// double obstacle_cost(int x, int y); // for later
		double quaternion_to_euler(geometry_msgs::Quaternion q); //

		cv::Mat map;
		
		// position
		geometry_msgs::Pose2D odometry;

		// map size
		const double m_height, m_width, m_resolution;
		const int m_grid_x, m_grid_y;
		geometry_msgs::Point raster_reference;

		// occupancy
		const int m_threshold;

		ros::Subscriber camera_input, laser_input, odometry_input;
		ros::ServiceServer successors, cell_to_real, real_to_cell, robot_position;
		ros::NodeHandle node;
		ros::Timer map_display_timer;		
};

#endif
