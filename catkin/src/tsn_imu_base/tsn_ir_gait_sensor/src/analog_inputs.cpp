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

void analog_inputs::assignFile(unsigned int pin)  {
	INDEX_FILE = "sys/bus/iio/devices/iio:device0/in_voltage%d_raw", pin;
	INDEX_VALUE = *INDEX_FILE;
}

// Check that we are getting values from the adc
bool analog_inputs::verifyADCPin(unsigned int pin)  {
	assignFile(pin);

	num = snprintf(buf, sizeof(buf), INDEX_VALUE);

	file = open(buf, O_RDONLY);

	if (file < 0)  {
		printf("Failed to open the bus.\n");
		return false;
	}
	else  {
		printf("Successfully conencted to bus.\n");
		return true;
	}

}

// Read the current ADC value from input pin
int analog_inputs::adcRead(unsigned int pin)
{
	assignFile(pin);
	num = 0;
	file = 0;

	num = snprintf(buf, sizeof(buf), INDEX_VALUE);

	file = open(buf, O_RDONLY);

	if (file < 0)  {
		printf("Failed to open the bus.\n");
	}

	read(file, &val, 3);
	close(file);

	return atoi(&val);
}