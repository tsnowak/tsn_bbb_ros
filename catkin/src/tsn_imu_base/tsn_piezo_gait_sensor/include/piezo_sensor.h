/* Code to record from piezoelectric sensor to function using ROS Indigo on a 
*  Black with Linux ARM OS running Kernel version 3.8.x..
*  Developer: Theodore Nowak BSc., Case Western Reserve University EECS Department
*  Date of Initiation: 03/23/2015
*
*  Functionality: Output std_msgs::Float32 containing analog input voltage from
*  desired pin. In future, attempt to make custom message type to output with this
*  value.

*  Application: Low cost gait tracking.
*/

#ifndef _PIEZO_SENSOR_H_
#define _PIEZO_SENSOR_H_

// ALWAYS need to include this when using ROS
#include <ros/ros.h> 
#include "Analoglib.h"
// some generically useful stuff to include...
#include <math.h>
// #include <stdlib.h>
#include <string>
// #include <stdio.h>
#include <tsn_bbb_msgs/Voltage.h>

const double UPDATE_RATE = 50; // desired publication rate of piezoelectric voltage data

class piezo_sensor  {
	public:
		piezo_sensor(ros::NodeHandle* nodehandle); // our main function needs to initialize a ROS node handler
		void initializePublishers();
		void fetchValues();

	private:
		ros::NodeHandle nh_;
		ros::Publisher piezo_sensor_publisher;
		tsn_bbb_msgs::Voltage data_out;  // variable name for our std_msgs::Float32 output
		float raw_data;  // variable we will store the raw 0-4096 data to
		FILE* file;  // File where our data is held/to be read from
};

#endif /* _PIEZO_SENSOR_H_ */