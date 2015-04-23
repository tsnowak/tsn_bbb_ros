# tsn_bbb_ros
This repository was created to allow for the publishing in ROS Indigo of data from the popular InvenSense
MPU-6050 6DOF IMU. Accessory requirements for our gait tracking application were two analog inputs for our
IR ground proximity sensor and Piezoelectric sensor for secondary vibration detection, and a GPIO output to trigger
high when IMU data is received to coordinate with VICON system we are using for verification.
  

#### People
Developed by [Theodore Nowak](https://github.com/RACKGNOME)

#### Purpose
This code was initially developed as a senior project at Case Western Reserve University to satisfy requirements
for BSc. candidates Alex Haufler and Theodore Nowak.

#### Current ROS Dependencies
The repository will be continuously updated as needed.  Please make sure your development system (and whatever robot you're trying to run our package on) has all of the dependencies listed below. Currently I have not written a setup file for this, but in the future I may. Manual steps are listed below.

* Acquire Beaglebone Black!

* Install Linux Arm OS via: 
	- http://www.armhf.com/boards/beaglebone-black/#trusty

* Ensure you are using Linux Arm Kernel 3.8.x. Install/Downgrade via:
	- sudo apt-get update
	- sudo apt-get install linux-image-3.8.13-bone68
	- sudo reboot 

* Install linux i2ctools via:
	- sudo apt-get update
	- sudo apt-get install i2ctools

* Install Device Tree Compiler via:
	- wget -c https://raw.github.com/RobertCNelson/tools/master/pkgs/dtc.sh
	- chmod +x dtc.sh
	- ./dtc.sh
	- "Note": There are other ways to install this, but Robert Nelson made it quick and easy

* Follow instructions to install your device tree overlay and create overlay via:
	- http://kilobaser.com/blog/2014-07-28-beaglebone-black-devicetreeoverlay-generator#dtogenerator

* Modify uENV.txt in /boot/ via:
	- Example-> optargs=capemgr.enable_partno=BB-ADC,bspm_P9_11_f
	- "Note": you need BB-ADC enabled to use analog inputs
	- "Note": bspm_P9_11_f is was generated by my generated device tree overlay below

* Change permissions of /sys/class/gpio/ folder via:
	- /sys/class/gpio$ sudo chmod -R DESIRED_PERMISSION
	- /sys/class/gpio/gpio#/ sudo chmod -R DESIRED_PERMISSION.

* Run code

*Each system is different so you may need to do some personal debugging. Figuring this out took me weeks to do. Hopefully this will help to minimize your install time to days, or even work perfectly for you. Cheers!
