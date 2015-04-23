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
ros_mpu6050::ros_mpu6050(ros::NodeHandle* nodehandle):nh_(*nodehandle)  { 
    
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


    // Using GPIO to rise whenever we fetch I2C information

    ROS_INFO("Exporting and setting direction of GPIO.");
    ROS_INFO("Exporting...");
    gpio_export(GPIO);  // Exporting our GPIO pin
    ROS_INFO("Setting Direction...");
    gpio_set_dir(GPIO, OUTPUT_PIN);  // Setting GPIO pin as output
    ROS_INFO("Setting Low...");
    gpio_set_value(GPIO, LOW);  // Initialize GPIO at 0V
    ROS_INFO("Getting Value...");
    gpio_get_value(GPIO, gpio_val);  // Verifying GPIO was set correctly

    if (gpio_val == 0)
        ROS_INFO("GPIO is set Low, success!");
    else
        ROS_WARN("GPIO is set High, error!");


    ROS_INFO("Done Initializing!"); 
}

// Simple function to set up all publishers (more can be added as desired)
void ros_mpu6050::initializePublishers()  {
    ROS_INFO("Initializing Publishers: imu_publisher, gpio_publisher");
    imu_publisher = nh_.advertise<sensor_msgs::Imu>("mpu_6050", 1, true); // publish IMU data in package sensor_msgs::Imu
    gpio_publisher = nh_.advertise<std_msgs::UInt8>("gpio_out", 1, true);  // publish current GPIO value
}

// retrieve IMU angular and linear accel values from IMU registers
void ros_mpu6050::fetchValues()  {
    // Retrieve IMU register gyro/accel data
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    data_out.header.stamp = ros::Time::now();  // time stamp the measurement

    data_out.linear_acceleration.x = (double)((ax+32767)*2*lima)/65534-lima;  // acc
    data_out.linear_acceleration.y = (double)((ay+32767)*2*lima)/65534-lima;
    data_out.linear_acceleration.z = (double)((az+32767)*2*lima)/65534-lima;

    data_out.angular_velocity.x = (double)((gx+32767)*2*limg)/65534-limg;  // gyro
    data_out.angular_velocity.y = (double)((gy+32767)*2*limg)/65534-limg;
    data_out.angular_velocity.z = (double)((gz+32767)*2*limg)/65534-limg;

    data_out.angular_velocity.x = data_out.angular_velocity.x*3.1415926/180;  // change to rad/s
    data_out.angular_velocity.y = data_out.angular_velocity.y*3.1415926/180;
    data_out.angular_velocity.z = data_out.angular_velocity.z*3.1415926/180;
  
   	imu_publisher.publish(data_out);  // publish
}

// set gpio high and publish 1
void ros_mpu6050::setGPIOHigh()  {
    gpio_set_value(GPIO, HIGH);
    gpio_data_out.data = 1;
}

// set gpio low and publish 0
void ros_mpu6050::setGPIOLow()  {
    gpio_set_value(GPIO, LOW);
    gpio_data_out.data = 0;
}

int main(int argc, char** argv)  {
    
    // ROS set-ups:
    ros::init(argc, argv, "rosmpu6050"); // node name

    ros::NodeHandle nh;  // create a node handle to pass to the class constructor

    ROS_INFO("main: instantiating an object of type ros_mpu6050");
    ros_mpu6050 rosmpu6050(&nh);  // instantiate an ros_mpu6050 object and pass in pointer to nodehandle for constructor to use
    ros::Rate sleep_timer(UPDATE_RATE/2);  // a timer for desired rate, 50Hz is a good speed. We set to half for 2 seperate sleeps

    ROS_INFO("Starting Data Recording From MPU6050");
    // loop to constantly "fetch" values from the MPU-6050
    while (ros::ok()) {

        rosmpu6050.fetchValues();
        rosmpu6050.setGPIOHigh();  // set GPIO pin high right after we get data
        ros::spinOnce();
        sleep_timer.sleep();  // leave HIGH and sleep half of our sleep time
        rosmpu6050.setGPIOLow();  // set GPIO pin low so we can get rising edge when we get data again 
        sleep_timer.sleep();  // finish sleep time
    }
    return 0;
} 

