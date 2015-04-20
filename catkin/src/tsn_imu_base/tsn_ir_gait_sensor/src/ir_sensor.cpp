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

#include <ir_sensor.h>

unsigned int pin;


ir_sensor::ir_sensor(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
	ROS_INFO("in class constructor of ir_sensor.cpp");

	initializePublishers();

	// Remove and set to default or desired if cin/cout is slowing you down
    std::cout<<"Starting IR Sensor..."<<std::endl;
         if (true) {
            std::cout<<"Confirm:  Enter IR Analog Pin: ";
            std::cin>>pin;   
        }

    ROS_INFO("Initialzing IR Sensor...");

    file = analog_inputs::returnFile(pin);
    size = analog_inputs::getFileSize(file);

    bool error = true;
    int count = 0;
    // ensure that our file exists and is working
    while (error && count <= 5)  {
    	if (analog_inputs::verifyADCPin(file, size))
    		error = false;
    	else 
    		ROS_ERROR("AIN%d File does not exist!", pin);
    		ros::Duration(0.5).sleep();
    		count ++;
    }
    

	ROS_INFO("Finished starting IR Sensor");
}

void ir_sensor::initializePublishers()
{
	ROS_INFO("Initializing Publishers: ir_sensor_publisher");
	ir_sensor_publisher = nh_.advertise<std_msgs::Float32>("ir_sensor", 1, true);
}

void ir_sensor::fetchValues()
{
	raw_data = analog_inputs::adcRead(file, size);
	data_out.data = 1.8*(raw_data/4096);
	ir_sensor_publisher.publish(data_out);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "irsensor");

	ros::NodeHandle nh;
	ir_sensor irsensor(&nh);
	ros::Rate sleep_timer(UPDATE_RATE);

	while (ros::ok())
	{
		irsensor.fetchValues();
		ros::spinOnce();
		sleep_timer.sleep();
	}
	return 0;
}
