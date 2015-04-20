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

#include <analog_inputs.h>

analog_inputs::analog_inputs()  {
}

// Check that we are getting values from the adc
bool analog_inputs::verifyADCPin(unsigned int pin)  {

	// int leftover;
	// char buf[128];
	char val[7];
	long int value_int = 0;
	unsigned int error_check;

	// leftover = snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/iio:device0/in_voltage%u_raw", pin);

	FILE* file = fopen("/sys/bus/iio/devices/iio:device0/in_voltage%u_raw", pin, "r");

	error_check = fread(&val, 6,6,file);
	
	if (error_check != 6)  {
		printf("Reading error.");
		return false;
	}
	else  {
		return true;
	}

}

// Read the current ADC value from input pin
int analog_inputs::adcRead(unsigned int pin)
{
	// int leftover;
	// char buf[128];
	char val[7];
	long int value_int = 0;

	// leftover = snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/iio:device0/in_voltage%u_raw", pin);

	FILE* file = fopen("/sys/bus/iio/devices/iio:device0/in_voltage%u_raw", pin, "r");

	fread(&val, 6,6,file);

	value_int = strtol(val,NULL,0);

	return value_int;
}