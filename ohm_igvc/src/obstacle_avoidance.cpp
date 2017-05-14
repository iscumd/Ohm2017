#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance");

    ros::NodeHandle n;
    ros::Subscriber obstacle_avoidance_sub = n.subscribe("robot_location", 1, Type.Missing)
    ros::Publisher obstaclePub = n.advertise<geometry_msgs::Twist>("obstacles",1000)

    ros::spin();
    return 0;
}
