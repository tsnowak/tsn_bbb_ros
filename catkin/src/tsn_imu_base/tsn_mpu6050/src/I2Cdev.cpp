// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2013-05-06 - add Francesco Ferrara's Fastwire v0.24 implementation with small modifications
//      2013-05-05 - fix issue with writing bit values to words (Sasquatch/Farzanegan)
//      2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                 - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//      2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//      2011-10-03 - added automatic Arduino version detection for ease of use
//      2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//      2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//      2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//      2011-08-02 - added support for 16-bit registers
//                 - fixed incorrect Doxygen comments on some methods
//                 - added timeout value for read operations (thanks mem @ Arduino forums)
//      2011-07-30 - changed read/write function structures to return success or byte counts
//                 - made all methods static for multi-device memory savings
//      2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*
===============================================
Although Jeff Rowberg generously created the inital library, edits were made 
so that it would work with the Beaglebone Black. A significant amount of this
editing was made by Nagavenkat Adurthi, a Ph.D. candidate at the University of 
Buffalo. His code can be found on his website. All other edits were made by
myself, Theodore Nowak BSc., Case Western Reserve University. Enjoy! 
* Insert non-liability rant from above. *
===============================================
*/

#include <I2Cdev.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// Change this depending on what i2c connection your IMU is using
// Check this using i2ctools (sudo i2cdetect -r 1,2, or 3)
#define BBB_I2C_file "/dev/i2c-0"

/** Default constructor.
 */
I2Cdev::I2Cdev() {
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable,
 * leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
    return readBytes(devAddr, regAddr, 1, data);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data) {
    return readWords(devAddr, regAddr, 1, data);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {


    int8_t count = 0;

       int file;
       char filename[40];
      // const gchar *buffer;
       //int addr = 0b00101001;        // The I2C address of the ADC

       sprintf(filename,BBB_I2C_file);
       if ((file = open(filename,O_RDWR)) < 0) {
           printf("Failed to open the bus.");
           close(file);
           return -1;
           /* ERROR HANDLING; you can check errno to see what went wrong */
          // exit(1);
       }

       if (ioctl(file,I2C_SLAVE,devAddr) < 0) {
           printf("Failed to acquire bus access and/or talk to slave.\n");
           /* ERROR HANDLING; you can check errno to see what went wrong */
           close(file);
           return -1;
         //  exit(1);
       }


       char buf[20] = {0};
       buf[0]=regAddr;
                   //go the register address by first writing to it
                   if (write(file,buf,1) != 1) {
                  /* ERROR HANDLING: i2c transaction failed */
                	   if (write(file,buf,1) != 1) {
                   printf("Failed to write(go to) the required register on the device.\n");
                	   }
                  // buffer = g_strerror(errno);
                  //	printf(buffer);
                  //	printf("\n\n");
                   close(file);
                   return -1;
                  	}


           if (read(file,buf,length) != length) {
               /* ERROR HANDLING: i2c transaction failed */
               printf("Failed to read the required no. of bytes from the i2c bus.\n");
             //  buffer = g_strerror(errno);
             //  printf(buffer);
             //  printf("\n\n");
               close(file);
               return -1;
           } else {
        	   for(;count<length;count++)
               {data[count] = (int)buf[count];}

           }
           close(file);
    return count;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data) {


    int8_t count = 0;

    int file;
           char filename[40];
   //        const gchar *buffer;
           //int addr = 0b00101001;        // The I2C address of the ADC

           sprintf(filename,BBB_I2C_file);
           if ((file = open(filename,O_RDWR)) < 0) {
               printf("Failed to open the bus.");
               close(file);
               return -1;
               /* ERROR HANDLING; you can check errno to see what went wrong */
             //  exit(1);
           }

           if (ioctl(file,I2C_SLAVE,devAddr) < 0) {
               printf("Failed to acquire bus access and/or talk to slave.\n");
               /* ERROR HANDLING; you can check errno to see what went wrong */
               close(file);
               return -1;
             //  exit(1);
           }


           char buf[20] = {0};
           buf[0]=regAddr;
            //go the register address by first writing to it
            if (write(file,buf,1) != 1) {
           /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to write(go to) the required register on the device.\n");
            //buffer = g_strerror(errno);
           //	printf(buffer);
           	//printf("\n\n");
            close(file);
            return -1;
           	}

               if (read(file,buf,2*length) != 2*length) {
                   /* ERROR HANDLING: i2c transaction failed */
                   printf("Failed to read the required no. of bytes/words from the i2c bus.\n");
                  // buffer = g_strerror(errno);
                //   printf(buffer);
                //   printf("\n\n");
                   close(file);
                   return -1;
               } else {
            	   for(;count<length;count++)
                   {data[count] = (int)((buf[2*count]<< 8)|buf[2*count+1]);}   //msb first and then lsb

               }
               close(file);
    return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {


	int file;
	           char filename[40];
	          // const gchar *buffer;
	           //int addr = 0b00101001;        // The I2C address of the ADC

	           sprintf(filename,BBB_I2C_file);
	           if ((file = open(filename,O_RDWR)) < 0) {
	               printf("Failed to open the bus.");
	               close(file);
	               return false;
	               /* ERROR HANDLING; you can check errno to see what went wrong */
	               //exit(1);
	           }

	           if (ioctl(file,I2C_SLAVE,devAddr) < 0) {
	               printf("Failed to acquire bus access and/or talk to slave.\n");
	               /* ERROR HANDLING; you can check errno to see what went wrong */
	               close(file);
	               return false;
	              // exit(1);
	           }


	           char buf[20] = {0};
	           buf[0]=regAddr;
	           int i;
	           for(i=0;i<length;i++)
	           {
	        	   buf[1+i]=data[i];
	           }
	           //float data;
	           //char channel;


	           //unsigned char reg = 0x10; // Device register to access
	           //buf[0] = reg;
	           //buf[0] = 0b11110000;

	               if (write(file,buf,length+1) != (length+1)) {
	                  /* ERROR HANDLING: i2c transaction failed */
	               printf("Failed to write the required no. of bytes to the i2c bus.\n");
	            //   buffer = g_strerror(errno);
	            //    printf(buffer);
	            //   printf("\n\n");
	               close(file);
	               return false;
	               }
	               close(file);
    return true;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {

	int file;
		           char filename[40];
		   //        const gchar *buffer;
		           //int addr = 0b00101001;        // The I2C address of the ADC

		           sprintf(filename,BBB_I2C_file);
		           if ((file = open(filename,O_RDWR)) < 0) {
		               printf("Failed to open the bus.");
		               close(file);
		               return false;
		               /* ERROR HANDLING; you can check errno to see what went wrong */
		               //exit(1);
		           }

		           if (ioctl(file,I2C_SLAVE,devAddr) < 0) {
		               printf("Failed to acquire bus access and/or talk to slave.\n");
		               /* ERROR HANDLING; you can check errno to see what went wrong */
		               close(file);
		               return false;
		              // exit(1);
		           }




					char buf[20] = {0};
		           buf[0]=regAddr;
		           int i;
		           for(i=0;i<2*length;i=i+1)
		           {
		        	   buf[1+2*i]=(uint8_t)(data[2*i] >> 8);  //first msb
		        	   buf[2+2*i]=(uint8_t)(data[2*i+1]);        // then lsb
		           }

		           if (write(file,buf,2*length+1) != (2*length+1)) {
		           	                  /* ERROR HANDLING: i2c transaction failed */
		           	               printf("Failed to write the required no. of words to the i2c bus.\n");
		           	         //      buffer = g_strerror(errno);
		           	         //       printf(buffer);
		           	          //     printf("\n\n");
		           	            close(file);
		           	               return false;

		           }
		           close(file);
		           return true;
}



