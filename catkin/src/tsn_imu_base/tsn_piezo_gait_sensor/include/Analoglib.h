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

#ifndef _ANALOGLIB_H_
#define _ANALOGLIB_H_
 
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

class Analoglib  {
	public:
		Analoglib();

		static FILE* openFile(unsigned int pin);  
		static bool verifyADCPin(FILE* file);
		static float adcRead(FILE* file);
		static void closeFile(FILE* file);
};

#endif /* _ANALOGLIB_H_ */
