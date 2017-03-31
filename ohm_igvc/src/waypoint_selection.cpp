#include "ros/ros.h"
#include "ohm_igvc/target.h"
#include "ohm_igvc/waypoint.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
using namespace std;

struct DMS
{
	double degrees;
	double minutes;
	double seconds;
};

vector<ohm_igvc::target> targetLocationList;

//because C++ doesn't have built in string splitting http://stackoverflow.com/a/236803
void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

double dmsConvert(double degrees, double minutes, double seconds) {
    double output;
	//ROS_DEBUG("%f %f' %.12f\" = ", degrees, minutes, seconds);
	output = abs((int)degrees) + (minutes/60) + (seconds/3600);
	if (degrees < 0) output *= -1;
	//ROS_DEBUG("%.12f\n", output);
	return output;
}

vector<ohm_igvc::target> ReadFile(string filename){
	string mode;
	string str;
	ifstream file;
	vector<ohm_igvc::target> navigationPoints = vector<ohm_igvc::target>();

	file.open(filename.c_str());
	if((file.rdstate() & std::ifstream::failbit) != 0){
		ROS_FATAL("Error opening %s", filename.c_str());
	}
	getline(file, mode); //decimal or dms
	while(getline(file, str)){
		ROS_DEBUG("Read file line: %s", str.c_str());
		if (str.substr(0,2) == "//") continue; //skip comments
		vector<string> lineFields = split(str, ','); //latitude,longitude
		if(lineFields.size() >= 2){
			ohm_igvc::target currentTarget;
			if(mode == "decimal"){
				currentTarget.latitude = atof(lineFields[0].c_str());
				currentTarget.longitude = atof(lineFields[1].c_str());
			}
			else if(mode == "dms"){ //Format: (degrees)d [(minutes)' [(seconds)"]]
				double degrees, minutes, seconds;
				sscanf(lineFields[0].c_str(), "%lfd %lf' %lf\"", &degrees, &minutes, &seconds);
				currentTarget.latitude = dmsConvert(degrees, minutes, seconds);
				sscanf(lineFields[1].c_str(), "%lfd %lf' %lf\"", &degrees, &minutes, &seconds);
				currentTarget.longitude = dmsConvert(degrees, minutes, seconds);
			}
			else{
				ROS_FATAL("Invalid waypoint mode.");
				break;
			}
			navigationPoints.push_back(currentTarget);
		}
	}
	file.close();

	return navigationPoints;
}

bool waypoint(ohm_igvc::waypoint::Request  &req,
              ohm_igvc::waypoint::Response &res){
	ROS_INFO("Received request: %i", req.ID);
	if (req.ID > -1 && req.ID < targetLocationList.size()){
		res.waypoint = targetLocationList[req.ID];
		ROS_INFO("Sent response: latitude=%f longitude=%f", res.waypoint.latitude, res.waypoint.longitude);
		return true;
	}
	else {
		return false;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "waypoint_selection");

	ros::NodeHandle n;
	
	std::string navigationFile;
	if (ros::param::get("navigationFile", navigationFile)){
		ROS_INFO("Using navigationFile %s", navigationFile.c_str());

		targetLocationList = ReadFile(navigationFile);

		for(int i = 0; i < targetLocationList.size(); i++){
			ROS_INFO("Target #%i: %f %f", i, targetLocationList[i].latitude, targetLocationList[i].longitude);
		}

		ros::ServiceServer service = n.advertiseService("waypoint", waypoint);

		ros::spin();
	}
	else{
		ROS_FATAL("No navigationFile parameter specified!");
	}
	
	return 0;
}