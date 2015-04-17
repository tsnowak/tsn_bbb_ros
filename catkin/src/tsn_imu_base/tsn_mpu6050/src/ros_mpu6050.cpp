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

#include <ros_mpu6050.h>

// global variable to use as the IMU address
int8_t imuid;

// constructor to initialize IMU at given address (verify address using i2cdetect -r 0,1, 
// or 2 ----- it should show a table of --'s or UU's for empty and N/A or in use I2C ports.
// Your IMU is shown as a number at the address it is using Ex. 60 and 8 for the default
// MPU-6050 address) and start up publisher and other ROS resources
ros_mpu6050::ros_mpu6050(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
    ROS_INFO("in class constructor of ros_mpu6050");

    initializePublishers();

    // Remove and set to default or desired if cin/cout is slowing you down
    std::cout<<"Starting MPU6050..."<<std::endl;
         if (true) {
            std::cout<<"Confirm:  Enter IMU Address: ";
            std::cin>>imuid;   
        }

    ROS_INFO("Starting MPU6050");
    // Type 0 to use the default address value for the MPU-6050 (Typically 68)
    if (imuid > 0)
        MPU6050 imu(imuid);
    else
        MPU6050 imu();

    ROS_INFO("Initializing MPU6050...");
    // Initialize the IMU setting the clock source, gyro-scale, accel-scale, sample rate 
    // (sample rate is 8kHz divided by # in setRate function -- see MPU6050.cpp), disable 
    // sleep mode, and sleep for 1 second at start-up
    imu.initialize();

    ROS_INFO("Done Initializing!"); 

}

// Simple function to set up all publishers (more can be added as desired)
void ros_mpu6050::initializePublishers()
{
    ROS_INFO("Initializing Publishers: imu_publisher");
    imu_publisher = nh_.advertise<sensor_msgs::Imu>("mpu_6050", 1, true); // publish IMU data in package sensor_msgs::Imu
}

// retrieve IMU angular and linear accel values from IMU registers
void ros_mpu6050::fetchValues()
{
    // Retrieve IMU register gyro/accel data
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    data_out.header.stamp = ros::Time::now();  // time stamp the measurement

    data_out.linear_acceleration.x =(double)((ax+32767)*2*lima)/65534-lima;  // acc
    data_out.linear_acceleration.y =(double)((ay+32767)*2*lima)/65534-lima;
    data_out.linear_acceleration.z =(double)((az+32767)*2*lima)/65534-lima;

    data_out.angular_velocity.x =(double)((gx+32767)*2*limg)/65534-limg;  // gyro
    data_out.angular_velocity.y =(double)((gy+32767)*2*limg)/65534-limg;
    data_out.angular_velocity.z =(double)((gz+32767)*2*limg)/65534-limg;

    data_out.angular_velocity.x =data_out.angular_velocity.x*3.1415926/180;  // change to rad/s
    data_out.angular_velocity.y =data_out.angular_velocity.y*3.1415926/180;
    data_out.angular_velocity.z =data_out.angular_velocity.z*3.1415926/180;
  
   	imu_publisher.publish(data_out);  // publish
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "rosmpu6050"); // node name

    ros::NodeHandle nh;  // create a node handle to pass to the class constructor

    ROS_INFO("main: instantiating an object of type ros_mpu6050");
    ros_mpu6050 rosmpu6050(&nh);  // instantiate an ros_mpu6050 object and pass in pointer to nodehandle for constructor to use
    ros::Rate sleep_timer(UPDATE_RATE);  // a timer for desired rate, 50Hz is a good speed

    ROS_INFO("Starting Data Recording From MPU6050");
    // loop to constantly "fetch" values from the MPU-6050
    while (ros::ok()) {

        rosmpu6050.fetchValues();
        
        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
} 

