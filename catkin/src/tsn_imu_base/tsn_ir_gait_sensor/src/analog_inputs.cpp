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

// Default empty constructor for analog_inputs library
analog_inputs::analog_inputs()  {
}

// Returns the opened file in which our analog data is stored
FILE* analog_inputs::openFile(unsigned int pin)  {
	char file_name[81];  // array of chars to later store value of pointed to file name

	// assign file_name the name of the file specified by /sys/bus/iio... etc 
	snprintf(file_name, 81, "/sys/bus/iio/devices/iio:device0/in_voltage%u_raw", pin);

	// Open the file at the location of file_name
	FILE* file = fopen(file_name, "r");

	return file;
}

// Check that we are getting values from the adc
bool analog_inputs::verifyADCPin(FILE* file)  {
	float value;  // Store voltage output

	// read file and assign output as float to value
	fscanf (file, "%f", &value);

	// Ensure we are getting valid readings
	if (value > 0)  
		return true;
	else
		return false;
}

// Close this reading of the file
void analog_inputs::closeFile(FILE* file)  {
	fclose(file);
}

// Read the current ADC value from input pin
float analog_inputs::adcRead(FILE* file)  {
	float value;  // variable to be assigned voltage value

	// reads file and assigns the data as type float to value
	fscanf (file, "%f", &value);

	return value;
}