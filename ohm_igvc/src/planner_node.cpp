#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <ohm_igvc/waypoint.h>
#include <ohm_igvc/get_successors.h>
#include <ohm_igvc/cell_to_real.h>
#include <ohm_igvc/real_to_cell.h>
#include <ohm_igvc/coordinate_convert.h>
#include <ohm_igvc/robot_position.h>
#include <ohm_igvc/pid_feedback.h>
#include <ohm_igvc/planned_path.h>

#include <boost/heap/priority_queue.hpp>
#include <vector>
#include <array>
#include <cmath>

struct Node {
    Node *parent;
    int x, y, which_child;
    float f, g, h, t;
	Node() {
		parent = nullptr;
		x = y = which_child = 0;
		f = g = h = 0.0;
	};
};

bool operator<(const Node &lhs, const Node &rhs) { return lhs.f < rhs.f; };
bool operator>(const Node &lhs, const Node &rhs) { return rhs < lhs; };
bool operator<=(const Node &lhs, const Node &rhs) { return !(lhs > rhs); };
bool operator>=(const Node &lhs, const Node &rhs) { return !(lhs < rhs); };
bool operator==(const Node &lhs, const Node &rhs) { return (lhs.x == rhs.x && lhs.y == rhs.y); };

class Planner {
    public:
        Planner(); // 
        std::vector<geometry_msgs::Pose2D> plan(int x, int y); // 
		void get_next_waypoint(int i); // 
		int get_robot_x() { return robot_position.x; };
		int get_robot_y() { return robot_position.y; };
		int get_goal_x() { return goal.x; };
		int get_goal_y() { return goal.y; };
		void get_robot_position(); //
		double distance_to_goal() { return distance(robot_position, goal); };
		double distance_to_last() { return distance(robot_position, last_in_path); };
		geometry_msgs::Point get_robot_real_position() { return cell_to_world(robot_position.x, robot_position.y); };
		void replan_path(const ros::TimerEvent &e);
			
    private:
		std::array<Node, 8> get_successors(int x, int y, Node &parent); // 
		geometry_msgs::Point cell_to_world(int x, int y);  //  

		double distance(Node first, Node second) { return std::hypot((second.x - first.x), (second.y - first.y)); }; // 
		double angular_distance(double a1, double a2) { return 180.0 - std::fabs(std::fabs(a1 - a2) - 180.0); }; // 

        double permissible_distance;
		double robot_theta;
		int robot_x, robot_y;

		double child_angle[8];

		Node goal;
		Node last_in_path;
		Node robot_position;

        ros::NodeHandle node;
        ros::ServiceClient map_get_successors, map_cell_to_world, map_world_to_cell, map_robot_position, coord_convert, waypoint_service;
};

Planner::Planner() {
	permissible_distance = 25;
	node.param("plan_distance", permissible_distance, permissible_distance);
	
	map_get_successors = node.serviceClient<ohm_igvc::get_successors>("get_successors", true);
	map_cell_to_world = node.serviceClient<ohm_igvc::cell_to_real>("cell_to_real", true);
	map_world_to_cell = node.serviceClient<ohm_igvc::real_to_cell>("real_to_cell", true);
	map_robot_position = node.serviceClient<ohm_igvc::robot_position>("get_robot_position", true);
	coord_convert = node.serviceClient<ohm_igvc::coordinate_convert>("coordinate_convert", true);
	waypoint_service = node.serviceClient<ohm_igvc::waypoint>("waypoint");
}

std::vector<geometry_msgs::Pose2D> Planner::plan(int x, int y) {
    boost::heap::priority_queue<Node> open, closed;
	std::vector<Node> path;
	std::vector<geometry_msgs::Pose2D> final_path;	

	path.reserve(100);
	final_path.reserve(40);
	
	Node start;
	start.parent = nullptr;
	start.f = 0;
	start.x = x;
	start.y = y;

	open.push(start);

	Node last_q = start;

	bool searching = true;

	while(!open.empty() && searching) {
		Node q = open.top();
		open.pop();
		
		std::array<Node, 8> successors = get_successors(q.x, q.y, q);

		for(auto child = successors.begin(); child != successors.end(); ++child) {
			if(child->parent == nullptr) continue;
			
			if(*child == goal) {
				goal.parent = child->parent;
				last_q = goal;                               
				searching = false;
				break;
			}

			child->g = q.g + distance(q, *child);
			child->h = distance(*child, goal);
			child->f = child->g + child->h + child->t;

			if(child->g > permissible_distance) continue;

			// sorry for this.

			bool skip = false;

			for(auto it = open.begin(); it != open.end(); ++it) { if(*child == *it && *child > *it) skip = true; }
			if(skip) continue;

			skip = false;

			for(auto it = closed.begin(); it != closed.end(); ++it) { if(*child == *it && *child > *it) skip = true; }
			if(skip) continue;

			// end sorry

			open.push(*child);
		}

		closed.push(q);

		last_q = q;                                                                                                                          
		
	}

	for(Node *n = &last_q; n->parent != nullptr; n = n->parent) { path.push_back(*n); }
	
	for(auto current = path.rbegin(), next = path.rbegin() + 1; next != path.rend(); ++current, ++next) {
		if(angular_distance(child_angle[current->which_child], child_angle[next->which_child]) >= 90.0) {
			geometry_msgs::Pose2D waypoint;
			geometry_msgs::Point real_coord = cell_to_world(current->x, current->y);

			waypoint.x = real_coord.x;
			waypoint.y = real_coord.y;
			waypoint.theta = child_angle[current->which_child];

			final_path.push_back(waypoint);
		}
	}

	last_in_path = path.front();

	return final_path;
}

void Planner::get_next_waypoint(int i) {
	ohm_igvc::waypoint req_wp;
	ohm_igvc::coordinate_convert req_conv;
	ohm_igvc::real_to_cell req_cell;

	req_wp.request.ID = i;
	
	waypoint_service.call(req_wp);

	req_conv.request.coordinate.latitude = req_wp.response.waypoint.latitude;
	req_conv.request.coordinate.longitude = req_wp.response.waypoint.longitude;


	coord_convert.call(req_conv);

	req_cell.request.real_coordinate.x = req_conv.response.coordinate.x;
	req_cell.request.real_coordinate.y = req_conv.response.coordinate.y;

	map_world_to_cell.call(req_cell);

	goal.x = req_cell.response.x;
	goal.y = req_cell.response.y;
}

std::array<Node, 8> Planner::get_successors(int x, int y, Node &parent) {
	std::array<Node, 8> successors;

	if(map_get_successors) {
		ohm_igvc::get_successors req;
		req.request.x = x;
		req.request.y = y;
		
		map_get_successors.call(req);
		
		for(int i = 0; i < 8; i++) {
			if(req.response.nodes[i].t_cost >= 0) {
				successors[i].parent = &parent;
			}
			
			successors[i].x = req.response.nodes[i].x;
			successors[i].y = req.response.nodes[i].y;	
			successors[i].which_child = i;		
			successors[i].t = req.response.nodes[i].t_cost;
		}
	}

	return successors;
}	

geometry_msgs::Point Planner::cell_to_world(int x, int y){
	/*** TODO: Implement reconnection logic for service calls ***/
	// if(map_cell_to_world) {
	ohm_igvc::cell_to_real req;

	req.request.x = x;
	req.request.y = y;

	map_cell_to_world.call(req);

	return req.response.real_coordinate;
	// }
}

void Planner::get_robot_position() {
	ohm_igvc::robot_position req;
	
	map_robot_position.call(req);

	robot_position.x = req.response.x;
	robot_position.y = req.response.y;
}

/* ------------------- // MAIN STUFF // -------------------- */

ohm_igvc::pid_feedback feedback;

void pid_feedback_callback(const ohm_igvc::pid_feedback::ConstPtr &fb) {
	feedback = *fb;
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
	double OP_SPEED = 0.25, PLAN_SPEED = 0.1, STOP = 0.0, SPEEDY_GONZALES = 0.5;
	node.param("planning_threshold", planning_threshold, planning_threshold);
	node.param("hit_threshold", hit_threshold, hit_threshold);
	node.param("planning_rate", planning_rate, planning_rate);
	node.param("operating_speed", OP_SPEED, OP_SPEED);
	node.param("planning_speed", PLAN_SPEED, PLAN_SPEED);
	node.param("top_speed", SPEEDY_GONZALES, SPEEDY_GONZALES);
	
	ros::Rate plan_rate(planning_rate);

	if(planning_threshold > 0) pid_feedback = node.subscribe<ohm_igvc::pid_feedback>("ohmPidFeedback", 1, &pid_feedback_callback);
	desired_speed = node.advertise<geometry_msgs::Twist>("/ohm/pathPlannerSpeed", 5);
	target = node.advertise<ohm_igvc::planned_path>("/ohm/pathPlanning", 5);

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
			
			current_path = planner.plan(planner.get_robot_x(), planner.get_robot_y());
			geometry_msgs::Point p = planner.get_robot_real_position();
			pid_path.lastTarget.latitude = p.x;
			pid_path.lastTarget.longitude = p.y;
			pid_path.currentTarget.latitude = current_path.front().x;
			pid_path.currentTarget.longitude = current_path.front().y;

			if(speed.linear.x = STOP) {
				speed.linear.x = OP_SPEED;
				desired_speed.publish(speed);
			}

			plan_rate.sleep();
		} else {
			if(planner.distance_to_goal() < hit_threshold) {
				// stop
				speed.linear.x = STOP;
				desired_speed.publish(speed);

				waypoint_id++;
				planner.get_next_waypoint(waypoint_id);

				current_path = planner.plan(planner.get_robot_x(), planner.get_robot_y());
				geometry_msgs::Point p = planner.get_robot_real_position();
				pid_path.lastTarget.latitude = p.x;
				pid_path.lastTarget.longitude = p.y;
				pid_path.currentTarget.latitude = current_path.front().x;
				pid_path.currentTarget.longitude = current_path.front().y;

				speed.linear.x = OP_SPEED;
				desired_speed.publish(speed);
			} else if(planner.distance_to_last() < planning_threshold) {
				speed.linear.x = PLAN_SPEED;
				desired_speed.publish(speed);

				current_path = planner.plan(planner.get_robot_x(), planner.get_robot_y());
				geometry_msgs::Point p = planner.get_robot_real_position();
				pid_path.lastTarget.latitude = p.x;
				pid_path.lastTarget.longitude = p.y;
				pid_path.currentTarget.latitude = current_path.front().x;
				pid_path.currentTarget.longitude = current_path.front().y;

				speed.linear.x = OP_SPEED;
				desired_speed.publish(speed);
			}

			if(feedback.targetDist < hit_threshold) {
				pid_path.lastTarget = pid_path.currentTarget;
				current_path.erase(current_path.begin());
				pid_path.currentTarget.latitude = current_path.front().x;
				pid_path.currentTarget.longitude = current_path.front().y;
			}
		}
	}

	return 0;
}
