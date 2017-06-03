#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose2D.h>
#include <ohm_igvc/coordinate_convert.h>
#include <vn300/Heading.h>
#include <vn300/Position.h>
#include <ohm_igvc/target.h>
#include <string>
#include <cmath>

#define DEG2RAD(x) ((3.14159265359 * x) / 180.00)

class odometry {
  public:
    odometry();
    void heading_callback(const vn300::Heading::ConstPtr &head);
    void position_callback(const vn300::Position::ConstPtr &pos);
    bool convert_callback(ohm_igvc::coordinate_convert::Request &rq, ohm_igvc::coordinate_convert::Response &rp);
	double gps_x(double lon) { return (K_EW * (lon - origin.longitude)); };
	double gps_y(double lat) { return (K_NS * (lat - origin.latitude)); };

  private:
    ros::Subscriber headingSub, positionSub;
    ros::Publisher pose;
    ros::ServiceServer coord_convert;
    ros::NodeHandle node;
    ohm_igvc::target origin;

    double K_NS, K_EW;

    geometry_msgs::Pose2D position;
    //origin - g
};

odometry::odometry() {
    std::string gps_heading = "/vn300/heading";
    std::string gps_position = "/vn300/position";
	K_NS = 111120.00;

	node.param("K_NS", K_NS, K_NS);
    node.param("origin_latitude", origin.latitude, 0.0);
    node.param("origin_longitude", origin.longitude, 0.0);
    node.param("gps_heading", gps_heading, gps_heading);
    node.param("gps_position", gps_position, gps_position);

	K_EW = K_NS * std::cos(DEG2RAD(origin.latitude));

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
    position.x = gps_x(pos->position[0]);
	position.y = gps_y(pos->position[1]);
	
	pose.publish(position);
}

bool odometry::convert_callback(ohm_igvc::coordinate_convert::Request &rq, ohm_igvc::coordinate_convert::Response &rp) {
	rp.coordinate.x = gps_x(rq.coordinate.latitude);
	rp.coordinate.y = gps_y(rq.coordinate.longitude);

	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "odometry");

    odometry node;

    ros::spin();

    return 0;
}
