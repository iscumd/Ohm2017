#include <ros/ros.h>
#include <ros/console.h>
#include <grid_map.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "grid_map");
	grid_map world;

	ros::spin();

	return 0;
}
