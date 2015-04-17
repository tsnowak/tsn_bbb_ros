/* Code to record from IR SENSOR to function using ROS Indigo on a Beaglebone
*  Black with Ubuntu ARM OS.
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

#include <ir_sensor.h>

ir_sensor::ir_sensor(ros::Nodehandle* nodehandle):nh_(*nodehandle)
{
	ROS_INFO("Starting IR Sensor");

	initializePublishers();

	ROS_INFO("Finished starting IR Sensor");
}

void ir_sensor::initializePublishers()
{
	ROS_INFO("Initializing Publishers: ir_sensor_publisher");
	ir_sensor_publisher = nh_.advertise<std_msgs::Float32>("ir_sensor", 1, true);
}

void ir_sensor::fetchValues()
{

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "irsensor");

	ros::Nodehandle nh;
	ir_sensor irsensor(&nh);
	ros::Rate sleep_timer(UPDATE_RATE);

	while (ros::ok())
	{

		fetchValues();
		ros::spinOnce();
		sleep_timer.sleep();
	}
}
