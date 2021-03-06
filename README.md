# ISEF-Project
A repository for TVC flight and launch code by Timothy Cai.

This Github repository contains all of the versions of flight, launch, and testing code that I've written for this project.
It's quite a mess, as I began this project with no knowledge of Github and its uses. The naming scheme is inconsistent, and 
the organization of the folders are a valiant attempt to create some sense of order out of the chaos that has descended upon
this first-time programmer. However, this repository is the most geniune representation of my work, and it is here, unfiltered.

The only relevant files are under **Flight/Flight_State_Machine_R3_V1**. However, you're welcome to look at all of the other files.
Many of them are outdated or different versions of the same base code for different tests, so read at your own risk. 
I recommend taking a look at Archive/Flight_State_Machine_V5, and comparing the change in code over the course of several months. 
They are both fully operational flight firmwares that were written about six months apart. 

This code can be run on the Kranz flight computer V2, with a BMI088 IMU and BMP280 barometer, on a Teensy3.6 running Teensyduino. 
In short, this code is customized to run in a very specific environment, so most likely can only be read.

This code relies on the following libraries:
<BMI088.h>
<TimerOne.h>
<Servo.h>
<SPI.h>
<SD.h>
<cstdio>
The majority of them are default Arduino libraries, and a few are customized for the hardware on the flight computer.
  
Enjoy!
