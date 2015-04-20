/* Implementation of MPU6050 and I2CDev cpp and h files to function using
*  ROS Indigo on a Beaglebone Black with Ubuntu ARM OS.
*  Developer: Theodore Nowak BSc., Case Western Reserve University EECS Department
*  Date of Initiation: 03/23/2015
*
*  Functionality: Output sensor_msgs imu.msg containing x,y,z linear acceleration,
*  and x,y,z angular acceleration. Possibly include orientation and methodology for
*  adequate zero update (ZUPDT) bias reset or other clever approach to removing bias
*  offset problem commonly associated with low cost IMUs.
*
*  Application: Low cost gait tracking.
*/

#ifndef _ROS_MPU6050_H_
#define _ROS_MPU6050_H_

// ALWAYS need to include this when using ROS
#include <ros/ros.h> 

// Include our modified MPU6050 library (which references I2Cdev)
#include <MPU6050.h">

// some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>

// message type used to communicate IMU data
#include <sensor_msgs/Imu.h>

const double UPDATE_RATE = 50; // desired publication rate of IMU data
const double limg = 1000;  // used to convert rotational accel to deg/s
const double lima = 2*9.8;  // used to convert linear accel to m/s^2

class ros_mpu6050  {
public:
	ros_mpu6050(ros::NodeHandle* nodehandle); // our main function needs to initialize a ROS node handler
	void initializePublishers();
	void fetchValues();
	
	MPU6050 imu;  // instantiation of object of class MPU6050

private:
	ros::NodeHandle nh_;
	ros::Publisher imu_publisher;
	sensor_msgs::Imu data_out;  // variable name for our sensor_msgs::Imu output
	int16_t ax, ay, az, gx, gy, gz;  // temp variables to store data from imu.getMotion6(...)

};

#endif /* _ROS_MPU6050_H_ */