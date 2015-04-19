/* Code to retrieve register values of the ADC to measure analog inputs. Intended
*  to function using ROS Indigo on a Beaglebone Black with Ubuntu ARM OS. Kernel
*  version 3.8.x REQUIRED with this library
*  Developer: Theodore Nowak BSc., Case Western Reserve University EECS Department
*  Date of Initiation: 04/14/2015
*
*  Functionality: Retrieve the data from the registers within the ADC of the Beagle-
*  Bone Black running Linux ARM Kernel 3.8.x.
*
*  Application: Low cost gait tracking.
*/

#ifndef _ANALOG_INPUTS_H_
#define _ANALOG_INPUTS_H_
 
#include <stdint.h>
#include <string>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/*
#define BBB_AIN0_FILE "sys/bus/iio/devices/iio:device0/in_voltage0_raw"
#define BBB_AIN1_FILE "sys/bus/iio/devices/iio:device0/in_voltage1_raw"
#define BBB_AIN2_FILE "sys/bus/iio/devices/iio:device0/in_voltage2_raw"
#define BBB_AIN3_FILE "sys/bus/iio/devices/iio:device0/in_voltage3_raw"
#define BBB_AIN4_FILE "sys/bus/iio/devices/iio:device0/in_voltage4_raw"
#define BBB_AIN5_FILE "sys/bus/iio/devices/iio:device0/in_voltage5_raw"
#define BBB_AIN6_FILE "sys/bus/iio/devices/iio:device0/in_voltage6_raw"
*/

class analog_inputs  {

	public:

		analog_inputs();

		static bool verifyADCPin(unsigned int pin);
		static int adcRead(unsigned int pin);
};

#endif /* _ANALOG_INPUTS_H_ */
