#include <ros/ros.h>
#include <ros/console.h>
#include <vn/sensors/sensors.h>
#include <boost/bind.hpp>
#include "ThreadPool.h"

#include <vn300/Pose.h>
#include <vn300/Velocities.h>
#include <vn300/Status.h>

// | 0  |  1  |  2  |  3  |  4 |  5  |  6  |  7  |  8  |  9  |  10  |  11  |  12  |  13  |  14  |  15  |
// | INS MODE |  ^  |     SENSOR ERROR     | N/A |  ^  |  ^  |                   N/A                   |       
//            | GPS FIX |                        |  ^  | GPS COMPASS |
//                                               | GPS HEADING INS |

// Bit masks to help extract the status info

#define INS_MODE_MASK 0x3
#define GPS_FIX_MASK 0x4
#define IMU_ERROR_MASK 0x10
#define MAGPRES_ERROR_MASK 0x20
#define GPS_ERROR_MASK 0x40
#define GPS_HEADING_INS_MASK 0x100
#define GPS_COMPASS_MASK 0x200

// class vn300_node
// Description: interfaces with vectornav sensor via vnproglib and outputs ros messages as they come in

class vn300_node {
	private:
		// vnproglib makes it impossible to use a class member as a packet handler easily.
		// so all these functions are friends to have access to the private members of a vn300_node but remain outside class
		// may use boost::bind at some point to get around that
		friend void vn300_packet_handler(void *userdata, vn::protocol::uart::Packet &p, size_t index);
		friend void vn300_error_handler(void *userdata, vn::protocol::uart::Packet &e, size_t index);
		friend std::string cookSensorError(vn::protocol::uart::SensorError e);

		vn::sensors::VnSensor sensor;

		nbsdx::concurrent::ThreadPool<5> workers; // 5 worker threads so we don't get a slowdown from serializing all the messages we get

		ros::Publisher pose;
		ros::Publisher velocity;
		ros::Publisher status;

		ros::NodeHandle node;

		std::string device;
		int rate;

		void setup(int pose_rate, int vel_rate, int status_rate);

	public:
		vn300_node();
		~vn300_node() {
			sensor.disconnect();
			workers.JoinAll();
		};

		bool ok() { return sensor.isConnected(); };

		// publishing wrappers allow us to add the message serialization to the thread pool

		void publish_pose_wrapper(vn300::Pose msg) { pose.publish(msg); };
		void publish_velocity_wrapper(vn300::Velocities msg) { velocity.publish(msg); };
		void publish_status_wrapper(vn300::Status msg) { status.publish(msg); };

		unsigned published() { return workers.JobsRemaining(); }; // debug function
};

/*
	cookSensorError
	parameters:
		e - integer representing the sensor error
	return: error message string
*/

std::string cookSensorError(vn::protocol::uart::SensorError e) {
	using namespace vn::protocol::uart;	
	switch(e) {
		case ERR_HARD_FAULT:
			return "HARDWARE FAULT";
		break;
		case ERR_SERIAL_BUFFER_OVERFLOW:
			return "SERIAL BUFFER OVERFLOW";
		break;
		case ERR_INVALID_CHECKSUM:
			return "INVALID CHECKSUM";
		break;          
		case ERR_INVALID_COMMAND:
			return "INVALID COMMAND";
		break;          
		case ERR_NOT_ENOUGH_PARAMETERS:
			return "NOT ENOUGH PARAMETERS";
		break;
		case ERR_TOO_MANY_PARAMETERS:
			return "TOO MANY PARAMETERS";
		break;       
		case ERR_INVALID_PARAMETER:
			return "INVALID PARAMETER";
		break;    
		case ERR_INVALID_REGISTER:
			return "INVALID REGISTER";
		break; 
		case ERR_UNAUTHORIZED_ACCESS:
			return "UNAUTHORIZED ACCESS";
		break; 
		case ERR_WATCHDOG_RESET:
			return "WATCHDOG RESET";
		break;   
		case ERR_OUTPUT_BUFFER_OVERFLOW:
			return "OUTPUT BUFFER OVERFLOW";
		break;   
		case ERR_INSUFFICIENT_BAUD_RATE:
			return "INSUFFICIENT BAUD RATE";
		break;
		case ERR_ERROR_BUFFER_OVERFLOW:
			return "ERROR BUFFER OVERFLOW";
		break;
		default:
			return "What error?";
	}
}

/* 
	vn300_packet_handler
	parameters:
		userdata - (stores pointer to vn300_node object)
		p - the packet received from the sensor
		index - packet count so far
	description:
		receives binary packets from sensor, extracts data, and sends job to thread pool to serialize the message and send it.
*/

void vn300_packet_handler(void *userdata, vn::protocol::uart::Packet &p, size_t index) {
	using namespace vn::sensors;
	using namespace vn::math;
	using namespace vn::protocol::uart;

	vn300_node *obj = (vn300_node *)userdata; // nasty, but it'll do

	if(p.type() == Packet::TYPE_BINARY) {
		//ROS_INFO_THROTTLE(2, "Binary packet recevied");
		if(p.isCompatible(COMMONGROUP_YAWPITCHROLL | COMMONGROUP_POSITION, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_POSU, ATTITUDEGROUP_YPRU, INSGROUP_NONE)) {	
			// p is a pose packet
			vn300::Pose msg;

			msg.header.stamp = ros::Time::now();

			vec3f ypr = p.extractVec3f();
			vec3d pos_lla = p.extractVec3d();
			vec3f pos_u = p.extractVec3f();
			vec3f ypr_u = p.extractVec3f();
			
			for(int i = 0; i < 3; i++) {
				msg.heading[i] = ypr[i];
				msg.position[i] = pos_lla[i];
				msg.heading[i + 3] = ypr_u[i];
				msg.position[i + 3] = pos_u[i];
			}

			obj->workers.AddJob(boost::bind(&vn300_node::publish_pose_wrapper, obj, msg));

		} else if(p.isCompatible(COMMONGROUP_ANGULARRATE | COMMONGROUP_VELOCITY, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_VELU, ATTITUDEGROUP_NONE, INSGROUP_NONE)) {			
			// p is a velocities packet
			vn300::Velocities msg;
				
			msg.header.stamp = ros::Time::now();
		
			vec3f angular_rate = p.extractVec3f();
			vec3f vel_ned = p.extractVec3f();
			float vel_u = p.extractFloat(); // note vel_u field in GPS group is NOT a vector of 3 floats, but a single float. attempting to extract a vec3f will cause the program to become nonresponsive.

			msg.velocity[3] = vel_u;

			for(int i = 0; i < 3; i++) {
				msg.velocity[i] = vel_ned[i];
				msg.angular[i] = angular_rate[i];
			}

			obj->workers.AddJob(boost::bind(&vn300_node::publish_velocity_wrapper, obj, msg));

		} else if(p.isCompatible(COMMONGROUP_INSSTATUS, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NUMSATS | GPSGROUP_FIX, ATTITUDEGROUP_NONE, INSGROUP_NONE)) {			
			// p is a status packet
			vn300::Status msg;

			msg.header.stamp = ros::Time::now();
		
			uint16_t ins_stat = p.extractUint16();
			uint8_t gps_num_sats = p.extractUint8();
			uint8_t gps_fix = p.extractUint8();

			// ins status bit field can be found at top of program

			msg.ins_mode = (ins_stat & INS_MODE_MASK);
			msg.usingGPSHeading = (bool)(ins_stat & GPS_HEADING_INS_MASK);
			msg.gpsCompassActive = (bool)(ins_stat & GPS_COMPASS_MASK);
			msg.imu_error = (bool)(ins_stat & IMU_ERROR_MASK);
			msg.magpres_error = (bool)(ins_stat & MAGPRES_ERROR_MASK);
			msg.num_sats = gps_num_sats;
			msg.fix = gps_fix;
			msg.gps_error = (bool)(ins_stat & GPS_ERROR_MASK);

			obj->workers.AddJob(boost::bind(&vn300_node::publish_status_wrapper, obj, msg));
			
		} else {
			//ROS_INFO("Unknown packet found");
		}
	} else {
		//ROS_DEBUG("Ascii packet received");
	}
}

/*
	vn300_error_handler
	parameters:	
		read vn300_packet_handler
*/

void vn300_error_handler(void *userdata, vn::protocol::uart::Packet &e, size_t index) {
	ROS_INFO("%s", cookSensorError(e.parseError()).c_str());
}

/*
	setup
	description: 
		sets up the binary registers to send out packets at at 10, 8, and 20 hz respectively. also registers the packet and error handlers with the library.
*/

void vn300_node::setup(int pose_rate, int vel_rate, int status_rate) {
	using namespace vn::sensors;
	using namespace vn::protocol::uart;

	BinaryOutputRegister pose_bor(
		ASYNCMODE_PORT1,
		20,
		COMMONGROUP_YAWPITCHROLL | COMMONGROUP_POSITION,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_POSU,
		ATTITUDEGROUP_YPRU,
		INSGROUP_NONE			
	);

	BinaryOutputRegister velocities_bor(
		ASYNCMODE_PORT1,
		50,
		COMMONGROUP_ANGULARRATE | COMMONGROUP_VELOCITY,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_VELU,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE
	);

	BinaryOutputRegister status_bor(
		ASYNCMODE_PORT1,
		20,
		COMMONGROUP_INSSTATUS,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NUMSATS | GPSGROUP_FIX,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE
	);

	sensor.writeAsyncDataOutputType(VNOFF);

	ROS_INFO("Connecting binary output 1 . . . ");
	sensor.writeBinaryOutput1(pose_bor);

	ROS_INFO("Connecting binary output 2 . . . ");
	sensor.writeBinaryOutput2(velocities_bor);

	ROS_INFO("Connecting binary output 3 . . . ");
	sensor.writeBinaryOutput3(status_bor);

	sensor.registerAsyncPacketReceivedHandler(this, vn300_packet_handler);
	sensor.registerErrorPacketReceivedHandler(NULL, vn300_error_handler);
}

/* 
	constructor
	description:
		attempts to connect to sensor. if sensor is not connected, throws vn::not_found. also advertises heading, position, and status.
*/

vn300_node::vn300_node() :
	device("/dev/ttyUSB0"),
	rate(115200)
{
	using namespace vn::sensors;
	using namespace vn::protocol::uart;

	int pose_hz = 10;
	int vel_hz = 8;
	int status_hz = 20;

	// params
	node.param("device", device, device); // for roslaunch files
	node.param("serial_rate", rate, rate);
	node.param("pose_refresh_rate", pose_hz, pose_hz);
	node.param("velocity_refresh_rate", vel_hz, vel_hz);
	node.param("status_refresh_rate", status_hz, status_hz);
	
	// setup
	
	try {
		sensor.connect(device, rate); // tried to prevent it from aborting, but couldn't find a workaround.
	} catch(vn::not_found &e) {}

	ROS_INFO("Connected to %s at %d baud", device.c_str(), rate);

	pose = node.advertise< vn300::Pose >("pose", true);
	velocity = node.advertise< vn300::Velocities >("velocities", true);
	status = node.advertise< vn300::Status >("status", true);

	setup(pose_hz, vel_hz, status_hz);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "vn300_node");
	vn300_node gps;
	ros::Rate r(10);

	while(ros::ok() && gps.ok()) { 
		//ROS_INFO("Publishing jobs remaining: %d", gps.published()); // debug
		r.sleep(); 
	}

	return 0;	
}
