#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include <vector>
#include <math.h>  
#include "lidarpoint.h"
#include "obstacle.h"
using namespace std;

//Global variables
ros::Publisher autoControlLogicPublisher = n.advertise<geometry_msgs::Twist>("direction", 1000)
ros::Publisher globalMappingPublisher = n.advertise<geometry_msgs::Twist>("obstacles", 1000)
float[] lidarData;
double theta;
vector<LidarPoint> lidarpoints;
vector<Obstacle> obstacles;
static double maxRadius = 10.0;
static double MM2M  = 0.001;
static int M2MM = 1000;
static double nonSeparationThresh = 200;
static int highThresh = 50;
static int lowThresh = 4;
int forgiveCount = 3;
int linkedCount;
int sumOfPoints;
int obsSizeNum;
bool isAlreadyLinking = false;
bool isThereAnObstacle = false;

//Callback Functions
void lidarPointsCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    lidarPoints = msg->ranges;
    theta = msg->angle_increment;

    convertAllPointsToCartesian();
    obstacleDetection();
}

//Computing Functions
void convertAllPointsToCartesian()
{
    int size = *(&lidarpoints + 1) - lidarpoints;

    double x = 0;
    double y = 0;
    double distaceFromRobot = 0;

    for(int i = 0; i < size; i++ )
    {
        x = cos(theta) * lidarData[i];
        y = sin(theta) * lidarData[i];
        distanceFromRobot = lidarData[i];

        LidarPoint  newPoint = new LidarPoint(x, y)
        lidarpoints.push_back(newPoint);
        x = 0;
        y = 0;
    }
}
double distanceCalculator(LidarPoint lidarPoint1, LidarPoint lidarPoint2)
{
    return sqrt(pow((lidarPoint2.x - lidarPoint1.x), 2)- pow((lidarPoint2.y - lidarPoint1.y), 2));
}
void linkPoint(double currPointDist, double twoPointsDist)
{
    linkedCount += 1;
    sum += currPointDist;
    obsSizeNum += twoPointsDist;
}
void addAndAnalyzeObstacle(int value, Obstacle& obstacle)
{
    double index = (value - linkedCount) / 2;
    double mag = sumOfPoints / linkedCount;
    bool isOutsideTheField = false;

    obstacle.x = mag * cos(theta);
    obstacle.y = mag * sin(theta);

    if (mag < maxRadius || linkedCount > highThresh || linkedCount < lowThresh)
    {
        clearState();
    }
    else
    {
        if(obstacle.x > 4.75 || obstacle.x < -1.750 || obstacle.y > 11.75 || obstacle.y < -2.75)// Needs to be rechecked
        {
            isOutsideTheField = true;
        }
        if(!isOutsideTheField)
        {
            if(mag <= 5) //if there is a close obstacle
            {
                isThereAnObstacle = true;
                ROS_INFO("Obstacle detected");
            }
            else
            {
                clearState();
            }
        }
    }
}
void clearState()
{
    sumOfPoints = 0;
    linkedCount = 0;
    isThereAnObstacle = false;
}
void obstacleDetection()
{
    for(int i = 360; i < lidarpoints.size() - 361; i++)
    {
        auto currentPoint = lidarpoints[count];
        bool isPointLinked = false;
        Obstacle obstacle;

        if(currentPoint < maxRadius)
        {
            for(int j = 0; j <= forgiveCount; j++ )
            {
                auto nextPoint = lidarpoints[i + 1];
                double pointsDistance = distanceCalculator(currentPoint, nextPoint);
                if(pointsDistance < nonSeparationThresh * j * MM2M)
                {
                    linkPoint(currentPoint.distanceFromRobot, pointsDistance)
                    isPointLinked = true;
                    if(!isAlreadyLinking)
                    { 
                        obstacle.objStartIndex = i;
                        isAlreadyLinking = true;
                    }
                    break;
                }
            }
        }
        if(isPointLinked == false)
        {
            if(isAlreadyLinking)
            {
                obstacle.objEndIndex = i;
                addAndAnalyzeObstacle(i, obstacle)
            }
            isAlreadyLinking = false;
            clearState();
        }
        else
        {
            i = i + j - 1;
            if(i > lidarpoints.size() - 361)
            {
                addAndAnalyzeObstacle(i, obstacle);
                clearState();
            }
        }
        
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance");
    bool keepMoving;
    
    ros::NodeHandle n;
    ros::Subscriber obstacle_avoidance_sub = n.subscribe("lidar_points", 1000, lidarPointsCallback)

    while(ros::ok())
    {
        geometry::Twist msg;

        if(isThereAnObstacle)
        {
            msg.linear.x = 0;
            msg.angular.y = 0;
        }

        autoControlLogicPublisher.publish(msg);
    }

    ros::spin();
    return 0;
}
