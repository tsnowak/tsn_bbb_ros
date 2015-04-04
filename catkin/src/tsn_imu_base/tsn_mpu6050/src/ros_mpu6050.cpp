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

#include "ros_mpu6050.h"

int imuid;

ros_mpu6050::ros_mpu6050(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of ros_mpu6050");

    initializePublishers();


    // Remove and set to default or desired if cin/cout is slowing you down
    std::cout<<"Starting MPU6050..."<<std::endl;
         if (true) {
            std::cout<<"Confirm:  Enter IMU Address: ";
            std::cin>>imuid;   
        }

    ROS_INFO("Starting MPU6050");
    // Alternative hard coded method of inputting IMU Address
    // imu.MPU6050(0);
    imu.MPU6050(imuid);

    ROS_INFO("Initializing MPU6050...");
    imu.initialize();

    ROS_INFO("Done Initializiing!"); 

}

//member helper function to set up publishers;
void ros_mpu6050::initializePublishers()
{
    ROS_INFO("Initializing Publishers: accls_publisher");
    imu_publisher = nh_.advertise<sensor_msgs::Imu>("mpu_6050", 1, true); // publish IMU data in package sensor_msgs::Imu
}

void ros_mpu6050::fetchValues()
{
	double temp_data;
	data = imu.getScaledaccgyro_timestamped(&temp_data);

	data_out.header.stamp = ros::Time::now();
	data_out.angular_velocity.x = data[1];
	data_out.angular_velocity.y = data[2];
	data_out.angular_velocity.z = data[3];
	data_out.linear_acceleration.x = data[4];
	data_out.linear_acceleration.y = data[5];
	data_out.linear_acceleration.z = data[6];
  
   	imu_publisher.publish(data_out);   
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "rosmpu6050"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type ros_mpu6050");
    ros_mpu6050 rosmpu6050(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate, e.g. 50Hz

    ROS_INFO:("Starting Data Recording From MPU6050");
    while (ros::ok()) {
        ros_mpu6050.fetchValues(); // compute and publish twist commands and cmd_vel and cmd_vel_stamped

        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
} 

