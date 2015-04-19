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

	int file, leftover;
	char buf[128];

	leftover = snprintf(buf, sizeof(buf), "sys/bus/iio/devices/iio:device0/in_voltage%u_raw", pin);

	printf("Leftover: %d", leftover);

	file = open(buf, O_RDONLY);

	if (file < 0)  {
		printf("Failed to open the bus.\n");
		return false;
	}
	else  {
		printf("Successfully connected to bus.\n");
		return true;
	}

}

// Read the current ADC value from input pin
int analog_inputs::adcRead(unsigned int pin)
{
	int file, leftover;
	char buf[128];
	char val[3];

	leftover = snprintf(buf, sizeof(buf), "sys/bus/iio/devices/iio:device0/in_voltage%u_raw", pin);

	file = open(buf, O_RDONLY);

	if (file < 0)  {
		printf("Failed to open the bus.\n");
	}

	read(file, &val, 3);
	close(file);

	return atoi(val);
}