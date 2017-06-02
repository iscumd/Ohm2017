#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose2D.h>
#include <ohm_igvc/pid_feedback.h>
#include <ohm_igvc/planned_path.h>
#include "planner.h"

// Globals
ohm_igvc::pid_feedback feedback;

void pid_feedback_callback(const ohm_igvc::pid_feedback::ConstPtr &fb) {
	feedback = fb;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "path_planner");
	Planner planner;
	ros::NodeHandle node;
	ros::Subscriber pid_feedback;
	ros::Publisher target, desired_speed;
	
	double planning_threshold = 15.0;
	double hit_threshold = 5.0;
	double planning_rate = 0.5;
	double OP_SPEED = 0.25, PLAN_SPEED = 0.1 STOP = 0.0, SPEEDY_GONZALES = 0.5;
	node.param("planning_threshold", planning_threshold, planning_threshold);
	node.param("hit_threshold", hit_threshold, hit_threshold);
	node.param("planning_rate", planning_rate, planning_rate);
	node.param("operating_speed", OP_SPEED, OP_SPEED);
	node.param("planning_speed", PLAN_SPEED, PLAN_SPEED);
	node.param("top_speed", SPEEDY_GONZALES, SPEED_GONZALES);
	
	ros::Rate plan_rate(planning_rate);

	if(planning_threshold > 0) pid_feedback = node.subscribe<ohm_igvc::pid_feedback>("ohmPidFeedback", 1, &pid_feedback_callback);
	desired_speed = node.advertise<geometry_msgs::Twist>("/ohm/pathPlannerSpeed", 5);
	target = node.advertise<ohm_igvc::path_planned>("/ohm/pathPlanning", 5);

	int waypoint_id = 0;
	bool replanned = false, waitOnHit = false;
	geometry_msgs::Twist speed;
	std::vector<geometry_msgs::Pose2D> current_path, next_path;
	ohm_igvc::planned_path pid_path;
	pid_path.dir = 1;

	while(ros::ok()) {
		ros::spinOnce();

		planner.get_robot_position();
		
		if(planning_threshold < 0) {
			if(planner.distance_to_goal() < hit_threshold) {
				// stop
				speed.linear.x = STOP;
				desired_speed.publish(speed);

				waypoint_id++;
				planner.get_next_waypoint(waypoint_id);
			}
			
			current_path = planner.plan();
			geometry_msgs::Point p = planner.get_robot_real_position();
			pid_path.last_target.latitude = p.x;
			pid_path.last_target.longitude = p.y;
			pid_path.current_target.latitude = current_path.front().x;
			pid_path.current_target.longitude = current_path.front().y;

			if(speed.linear.x = STOP) {
				speed.linear.x = OP_SPEED;
				desired_speed.publish(speed);
			}

			plan_rate.sleep();
		} else (planning_threshold > 0) {
			if(planner.distance_to_goal() < hit_threshold) {
				// stop
				speed.linear.x = STOP;
				desired_speed.publish(speed);

				waypoint_id++;
				planner.get_next_waypoint(waypoint_id);

				current_path = planner.plan();
				geometry_msgs::Point p = planner.get_robot_real_position();
				pid_path.last_target.latitude = p.x;
				pid_path.last_target.longitude = p.y;
				pid_path.current_target.latitude = current_path.front().x;
				pid_path.current_target.longitude = current_path.front().y;

				speed.linear.x = OP_SPEED;
				desired_speed.publish(speed);
			} else if(planner.distance_to_last() < planning_threshold) {
				speed.linear.x = PLAN_SPEED;
				desired_speed.publish(speed);

				current_path = planner.plan();
				geometry_msgs::Point p = planner.get_robot_real_position();
				pid_path.last_target.latitude = p.x;
				pid_path.last_target.longitude = p.y;
				pid_path.current_target.latitude = current_path.front().x;
				pid_path.current_target.longitude = current_path.front().y;

				speed.linear.x = OP_SPEED;
				desired_speed.publish(speed);
			}

			if(feedback.targetDist < hit_threshold) {
				pid_path.last_target = pid_path.current_target;
				current_path.erase(current_path.begin());
				pid_path.current_target.latitude = current_path.front().x;
				pid_path.current_target.longitude = current_path.front().y;
			}
		}
	}

	return 0;
}
