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

#include <ros_mpu6050.h>

int8_t imuid;

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
    if (imuid > 0)
        MPU6050 imu(imuid);
    else
        MPU6050 imu();

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

    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    data_out.header.stamp = ros::Time::now();; //time stamp the measurement

    data_out.linear_acceleration.x =(double)((ax+32767)*2*lima)/65534-lima;  //acc
    data_out.linear_acceleration.y =(double)((ay+32767)*2*lima)/65534-lima;
    data_out.linear_acceleration.z =(double)((az+32767)*2*lima)/65534-lima;

    data_out.angular_velocity.x =(double)((gx+32767)*2*limg)/65534-limg; //gyro
    data_out.angular_velocity.y =(double)((gy+32767)*2*limg)/65534-limg;
    data_out.angular_velocity.z =(double)((gz+32767)*2*limg)/65534-limg;

    data_out.angular_velocity.x =data_out.angular_velocity.x*3.1415926/180;   //change to rad/s
    data_out.angular_velocity.y =data_out.angular_velocity.y*3.1415926/180;
    data_out.angular_velocity.z =data_out.angular_velocity.z*3.1415926/180;
  
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

        rosmpu6050.fetchValues(); // compute and publish twist commands and cmd_vel and cmd_vel_stamped

        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
} 

