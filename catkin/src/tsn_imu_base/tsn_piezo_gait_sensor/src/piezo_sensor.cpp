/* Code to record from piezoelectric sensor to function using ROS Indigo on a 
*  Beaglebone Black with Ubuntu ARM OS.
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

#include <piezo_sensor.h>

piezo_sensor::piezo_sensor(ros::Nodehandle* nodehandle):nh_(*nodehandle)
{
	ROS_INFO("Starting Piezoelectric Sensor");

	initializePublishers();

	ROS_INFO("Finished starting Piezoelectric Sensor");
}

piezo_sensor::initializePublishers()
{
	ROS_INFO("Initializing Publisher: piezo_sensor_publisher");
	piezo_sensor_publisher = nh_.advertise<std_msgs::Float32>("piezo_sensor", 1, true);
}

piezo_sensor::fetchValues()
{

}

int main (int argc, char** argv)
{
	ros::init(argc, argc, "piezosensor");

	ros::Nodehandle nh_;
	piezo_sensor piezosensor(&nh);
	ros::Rate sleep_timer(UPDATE_RATE);

	while (ros::ok())
	{
		fetchValues();
		ros::SpinOnce();
		sleep_timer.sleep();
	}
}