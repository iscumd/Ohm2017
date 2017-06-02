#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose2D.h>
#include <ohm_igvc/coordinate_convert.h>
#include <vn300/Heading.h>
#include <vn300/Position.h>
#include <ohm_igvc/target.h>
#include <string>
#include <cmath>

class odometry {
  public:
    odometry();
    void heading_callback(const vn300::Heading::ConstPtr &head);
    void position_callback(const vn300::Position::ConstPtr &pos);
    bool convert_callback(ohm_igvc::coordinate_convert::Request &rq, ohm_igvc::coordinate_convert::Response &rp);

  private:
    ros::Subscriber headingSub, positionSub;
    ros::Publisher pose;
    ros::ServiceServer coord_convert;
    ros::NodeHandle node;
    ohm_igvc::target origin;
    // target msg: float64 latitude, float64 longitude
    geometry_msgs::Pose2D position;
    //origin - g
};

odometry::odometry() {
    std::string gps_heading = "/vn300/heading";
    std::string gps_position = "/vn300/position";

    node.param("origin_latitude", origin.latitude, 0.0);
    node.param("origin_longitude", origin.longitude, 0.0);
    node.param("gps_heading", gps_heading, gps_heading);
    node.param("gps_position", gps_position, gps_position);

    headingSub = node.subscribe<vn300::Heading>(gps_heading, 5, &odometry::heading_callback, this);
    positionSub = node.subscribe<vn300::Position>(gps_position, 5, &odometry::position_callback, this);

	coord_convert = node.advertiseService("coordinate_convert", &odometry::convert_callback, this);

    pose = node.advertise<geometry_msgs::Pose2D>("/ohm/odom", 5);
    position.x = 0.0;
    position.y = 0.0;
    position.theta = 0.0;
}

void odometry::heading_callback(const vn300::Heading::ConstPtr &head) {
    position.theta = head->heading[0]; 
    pose.publish(position);

}

void odometry::position_callback(const vn300::Position::ConstPtr &pos) {
    position.x = (pos->position[0] - origin.latitude);
	position.y = (pos->position[1] - origin.longitude);
	
	pose.publish(position);
}

bool odometry::convert_callback(ohm_igvc::coordinate_convert::Request &rq, ohm_igvc::coordinate_convert::Response &rp) {
	rp.coordinate.x = (rq.coordinate.latitude - origin.latitude);
	rp.coordinate.y = (rq.coordinate.longitude - origin.longitude);

	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "odometry");

    odometry node;

    ros::spin();

    return 0;
}