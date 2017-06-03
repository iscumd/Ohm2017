#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "isc_shared/joystick.h"
#include "isc_shared/wheel_speeds.h"

#include <serial/serial.h>
#include <sstream>
#include <string>

using std::string;
using std::stringstream;
using serial::Serial;

bool startButtonDown = false;
bool autoMode = false;
ros::Publisher wheelSpeedPub;

std::string arduinoPort;
serial::Serial *arduinoSerialPort;

void arduinoDisconnect(){
	if(arduinoSerialPort != NULL) {
		delete arduinoSerialPort;
		arduinoSerialPort = NULL;
	}
}

void arduinoConnect(){
	if(arduinoPort.empty()){
		ROS_ERROR("Arduino serial port name is empty.");
		return;
	}

	arduinoDisconnect();

	arduinoSerialPort = new Serial();
	arduinoSerialPort->setPort(arduinoPort);
 	arduinoSerialPort->setBaudrate(9600);
  	arduinoSerialPort->setBytesize(serial::eightbits);
  	arduinoSerialPort->setParity(serial::parity_even);
	serial::Timeout to = serial::Timeout::simpleTimeout(10);
	arduinoSerialPort->setTimeout(to);

	arduinoSerialPort->open();
	ROS_INFO("Connected to Arduino.");
}

void arduinoSendCommand(string command){
	ROS_INFO("Sending Arduino commend: %s", command.c_str());
	arduinoSerialPort->write(command+"\r");
}

void updateDriveMode(){
	ROS_INFO("Drive Mode Control: Switching to %s mode.", autoMode ? "AUTO" : "MANUAL");
	if(autoMode){
		arduinoSendCommand("A");
		ros::param::set("/drive_mode", "auto");
	}
	else {
		arduinoSendCommand("M");
		ros::param::set("/drive_mode", "manual");
	}
}

void joystickCallback(const isc_shared::joystick::ConstPtr& joy){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */

	/* We track the button state to ensure you can't 
	accidentally hold down Start and press another 
	button to toggle the mode */
	if(joy->Start){ //The Start button has been pressed
		startButtonDown = true;
	}
	if(startButtonDown && !joy->Start){ //The Start button has been released
		startButtonDown = false;
		autoMode = !autoMode;
		updateDriveMode();
	}
}

void manualCallback(const geometry_msgs::Twist::ConstPtr& msg){
	if(!autoMode){
		float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;

		leftWheelSpeed = (msg->linear.x - msg->angular.z);
		rightWheelSpeed = (msg->linear.x + msg->angular.z);

		isc_shared::wheel_speeds msg;
		msg.left = leftWheelSpeed;
		msg.right =  rightWheelSpeed;
		wheelSpeedPub.publish(msg);
	}
}

void autoCallback(const geometry_msgs::Twist::ConstPtr& msg){
	if(autoMode){
		float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;

		leftWheelSpeed = (msg->linear.x - msg->angular.z);
		rightWheelSpeed = (msg->linear.x + msg->angular.z);

		isc_shared::wheel_speeds msg;
		msg.left = leftWheelSpeed;
		msg.right =  rightWheelSpeed;
		wheelSpeedPub.publish(msg);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "ohm_drive_mode_control");

	ros::NodeHandle n;

	n.param("arduino_serial_port", arduinoPort, std::string("/dev/ttyACM0"));
	arduinoConnect();

	updateDriveMode();

	wheelSpeedPub = n.advertise<isc_shared::wheel_speeds>("wheelSpeeds", 5);

	ros::Subscriber joystickSub = n.subscribe("joystick", 5, joystickCallback);
	ros::Subscriber manualSub = n.subscribe("manualControl", 5, manualCallback);
	ros::Subscriber autoSub = n.subscribe("autoControl", 5, autoCallback);

	ros::spin();

	//node shutdown
	arduinoDisconnect();
	
	return 0;
}