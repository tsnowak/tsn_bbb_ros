/* Implementation of MPU6050 and I2CDev cpp and h files to function using
*  ROS Indigo on Beaglebone Black with Ubuntu ARM OS.
*  Developer: Theodore Nowak, Case Western Reserve University EECS Department
*  Date of Initiation: 03/23/2015
*
*  Functionality: Output sensor_msgs imu.msg containing x,y,z linear acceleration,
*  and x,y,z angular acceleration. Possibly include orientation and methodology for
*  adequate zero update (ZUPDT) bias reset or other clever approach to removing bias
*  offset problem.
*
*  Application: Low cost gait tracking.
*/

#ifndef ROS_MPU6050_H_
#define ROS_MPU6050_H_

#include "MPU6050.h"

// some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this when using ROS

// standard messages that we may need; include more if necessary
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// message type used to communicate IMU data
#include <sensor_msgs/Imu.h>

// various geometry messages that may be of use to us
/*#include <geometry_msgs/Quaternion>
#include <geometry_msgs/Vector3>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

// Eigen is useful for linear algebra
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
*/
const double UPDATE_RATE = 50.0; // desired publication rate of IMU data

class ros_mpu6050
{
public:
	ros_mpu6050(ros::NodeHandle* nodehandle); // our main function needs to initialize a ROS node handler
	void initializePublishers();
	void fetchValues();

	MPU6050 imu;
private:
	ros::NodeHandle nh_;

	ros::Publisher imu_publisher;

	sensor_msgs::Imu data_out;

	double *data;

};

#endif