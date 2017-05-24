#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <chrono>
#include <string>

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;

ros::Publisher autoPub;
Clock::time_point lastOverride = Clock::now();
double overrideDuration

void pidCallback(const geometry_msgs::Twist::ConstPtr& msg){
    if(std::chrono::duration_cast<milliseconds>(Clock::now() - lastOverride).count() > overrideDuration){
        autoPub.publish(*msg);

        ROS_INFO("Auto Control: pid linear.x=%f angular.z=%f", msg.linear.x, msg.angular.z);
    }
}

void overrideCallback(const geometry_msgs::Twist::ConstPtr& msg){
    lastOverride = Clock::now();
	autoPub.publish(*msg);

	ROS_INFO("Auto Control: override linear.x=%f angular.z=%f", msg.linear.x, msg.angular.z);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "auto_control_logic");

	ros::NodeHandle n;

    n.param("obstacleOverrideDuration", overrideDuration, 1.0);

	autoPub = n.advertise<geometry_msgs::Twist>("autoControl", 5);

	ros::Subscriber pidSub = n.subscribe("ohmPid", 5, pidCallback);
    ros::Subscriber overrideSub = n.subscribe("avoidanceOverride", 5, overrideCallback);

	ros::spin();
	
	return 0;
}