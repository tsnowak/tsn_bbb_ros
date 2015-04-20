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

char* analog_inputs::returnFile(unsigned int pin)  {
	char file_name[81];
	snprintf(file_name, 81, "/sys/bus/iio/devices/iio:device0/in_voltage%u_raw", pin);
	return file_name;
}

FILE* analog_inputs::openFile(char* file_name)  {
	FILE* file = fopen(file_name, "r");
	return file;
}

// Check that we are getting values from the adc
bool analog_inputs::verifyADCPin(FILE* file)  {
	
	/*

	// char val[7];
	char * val;
	long int value_int = 0;
	unsigned int error_check;

	val = (char*) malloc (sizeof(char)*size);

	rewind(file);
	// error_check = fread(&val, 6,6,file);
	
	fread(val, 2, size, file);

	/*if (error_check != 6)  {
		printf("Reading error.");
		return false;
	}
	else  {
		return true;
	}*/

	float value;

	fscanf (file, "%f", &value);

	return true;

}

void analog_inputs::closeFile(FILE* file)
{
	fclose(file);
}

// Read the current ADC value from input pin
float analog_inputs::adcRead(FILE* file)
{
	float value;

	fscanf (file, "%f", &value);

	/*
	rewind(file);
	debug = fread(val, 2, size, file);
	//perror();
	printf("Debug: %u", debug);
	value_int = strtol(val,NULL,2);
	*/
	return value;
}