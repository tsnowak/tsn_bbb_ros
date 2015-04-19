/* Code to record from IR SENSOR to function using ROS Indigo on a Beaglebone
*  Black with Ubuntu ARM OS.
*  Developer: Theodore Nowak BSc., Case Western Reserve University EECS Department
*  Date of Initiation: 03/23/2015
*
*  Functionality: Output std_msgs::Float32 containing analog input voltage from
*  desired pin. In future, attempt to make custom message type to output with this
*  value.
*
*  Application: Low cost gait tracking.
*/

#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_

// ALWAYS need to include this when using ROS
#include <ros/ros.h> 

// some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>

#include <std_msgs/Float32.h>

const double UPDATE_RATE = 50; // desired publication rate of ir sensor data

class ir_sensor
{
public:
	
	ir_sensor(ros::NodeHandle* nodehandle); // our main function needs to initialize a ROS node handler
	void initializePublishers();
	void fetchValues();

private:

	ros::NodeHandle nh_;
	ros::Publisher ir_sensor_publisher;
	std_msgs::Float32 data_out;  // variable name for our std_msgs::Float32 output
	int raw_data;
};

#endif /* _IR_SENSOR_H_ */