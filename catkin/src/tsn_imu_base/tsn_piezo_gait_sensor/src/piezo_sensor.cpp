/* Code to record from piezoelectric sensor to function using ROS Indigo on a 
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

#include <piezo_sensor.h>

// global variable of pin to which our Piezo Sensor is connected to
unsigned int pin; 

// constructor for piezo sensor
piezo_sensor::piezo_sensor(ros::NodeHandle* nodehandle):nh_(*nodehandle)  {

    ROS_INFO("in class constructor of piezo_sensor.cpp");

    initializePublishers();

    // Input the pin at which the Piezo Sensor is located
    std::cout<<"Starting Piezo Sensor..."<<std::endl;
         if (true) {
            std::cout<<"Confirm:  Enter Piezo Analog Pin: ";
            std::cin>>pin;   
        }

    ROS_INFO("Initialzing Piezo Sensor...");

    bool error = true;  // error tracker to hop out of while loop
    int count = 0;  // 5 strikes and you have a real problem, in case first time doesn't read quite right

    // ensure that our file exists and is working
    while (error && count < 5)  {
        file = Analoglib::openFile(pin);  // open our file for reading

        // try to read from it
        if (Analoglib::verifyADCPin(file))  {  
            ROS_INFO("Successfully read from file for Analog Pin %u!", pin);
            error = false;
        }
        else  {
            ROS_ERROR("Error reading file for in_voltage%u_raw", pin);
            ros::Duration(0.5).sleep();  // Don't eat up cpu, just take a short nap and try again
            count ++;
        }
        Analoglib::closeFile(file);  // always make sure to close the file after reading
    }
    

    ROS_INFO("Finished starting Piezo Sensor");
}

// initialize publishers
void piezo_sensor::initializePublishers()  {
    ROS_INFO("Initializing Publishers: piezo_sensor_publisher");
    piezo_sensor_publisher = nh_.advertise<tsn_bbb_msgs::Voltage>("piezo_sensor", 1, true);
}

// function to fetch and publish ADC values for certain pin
void piezo_sensor::fetchValues() {
    file = Analoglib::openFile(pin);  // open the file with the ADC data for our pin inside

    raw_data = Analoglib::adcRead(file);  // read this data and store it in our local var raw_data

    data_out.header.stamp = ros::Time::now();  // time stamp the measurement
    
    data_out.data = 1.8*(raw_data/4096);  // convert this data to the proper format

    piezo_sensor_publisher.publish(data_out);  // publish this data in our desired msg type (std_msgs::Float32)

    Analoglib::closeFile(file);  // and per usual remember to close the file we read from
}

int main (int argc, char** argv)  {
    ros::init(argc, argv, "piezosensor");  // initialize our ros node "piezosensor"

    ros::NodeHandle nh;
    piezo_sensor piezosensor(&nh);  // our node hungers for a nodehandler
    ros::Rate sleep_timer(UPDATE_RATE);  // create a sleep timer with rate UPDATE_RATE

    // fetch, update, sleep, repeat
    while (ros::ok())  {
        piezosensor.fetchValues();
        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
}