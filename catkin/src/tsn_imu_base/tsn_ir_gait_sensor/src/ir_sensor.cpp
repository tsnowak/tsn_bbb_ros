/* Code to record from IR SENSOR to function using ROS Indigo on a Beaglebone
*  Black with Linux ARM OS running Kernel version 3.8.x.
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

// global variable of pin to which our IR Sensor is connected to
unsigned int pin;

// constructor for ir_sensor
ir_sensor::ir_sensor(ros::NodeHandle* nodehandle):nh_(*nodehandle)  {

	ROS_INFO("in class constructor of ir_sensor.cpp");

	initializePublishers();

	// Input the pin at which the IR Sensor is located
    std::cout<<"Starting IR Sensor..."<<std::endl;
         if (true) {
            std::cout<<"Confirm:  Enter IR Analog Pin: ";
            std::cin>>pin;   
        }

    ROS_INFO("Initialzing IR Sensor...");

    bool error = true;  // error tracker to hop out of while loop
    int count = 0;  // 5 strikes and you have a real problem, in case first time doesn't read quite right

    // ensure that our file exists and is working
    while (error && count < 5)  {
    	file = analog_inputs::openFile(pin);  // open our file for reading

    	// try to read from it
    	if (analog_inputs::verifyADCPin(file))  {  
    		ROS_INFO("Successfully read from file for Analog Pin %u!", pin);
    		error = false;
    	}
    	else  {
    		ROS_ERROR("Error reading file for in_voltage%u_raw", pin);
    		ros::Duration(0.5).sleep();  // Don't eat up cpu, just take a short nap and try again
    		count ++;
    	}
    	analog_inputs::closeFile(file);  // always make sure to close the file after reading
    }
    

	ROS_INFO("Finished starting IR Sensor");
}

// initialize all of our publishers in one fell swoop
void ir_sensor::initializePublishers()  {
	ROS_INFO("Initializing Publishers: ir_sensor_publisher");
	ir_sensor_publisher = nh_.advertise<std_msgs::Float32>("ir_sensor", 1, true);
}

// function to fetch and publish ADC values for certain pin
void ir_sensor::fetchValues()  {
	file = analog_inputs::openFile(pin);  // open the file with the ADC data for our pin inside

	raw_data = analog_inputs::adcRead(file);  // read this data and store it in our local var raw_data

	data_out.data = 1.8*(raw_data/4096);  // convert this data to the proper format

	ir_sensor_publisher.publish(data_out);  // publish this data in our desired msg type (std_msgs::Float32)

	analog_inputs::closeFile(file);  // and per usual remember to close the file we read from
}

int main (int argc, char** argv)  {
	ros::init(argc, argv, "irsensor");  // initialize our ros node "irsensor"

	ros::NodeHandle nh;
	ir_sensor irsensor(&nh);  // our node hungers for a nodehandler
	ros::Rate sleep_timer(UPDATE_RATE);  // create a sleep timer with rate UPDATE_RATE

	// fetch, update, sleep, repeat
	while (ros::ok())  {
		irsensor.fetchValues();
		ros::spinOnce();
		sleep_timer.sleep();
	}
	return 0;
}