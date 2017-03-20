/*
 * Node to monitor and publish multi-proximity sensor data from mark-toys multi-proximity sensor
 *
 * This module implements a ROS node (process) that monitors the proximity sensors and sends to ros topic
 *
 * Output is in a custom message with one millimeter measurement for up to 8 sensors 
 *
 * Author:    Mark Johnston
 * Creation:  20170315    Initial creation of this module in rosbits repository
 *
 */

#define  THIS_NODE_NAME    "prox_multi_sensor"   // The Name for this ROS node

/************************************************************************************
 * The ROS Code usage requires the following comments be pressent:
 *
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

// These next few are for I2C and ioctls and file opens
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <termios.h>    // POSIX terminal control definitions

using namespace std;

// The Standard ROS sensor_msgs/Range.msg
#include <sensor_msgs/Range.h>

// These includes are very specific to our hardware and system software internals
#include <ros_bits/prox_multi_sensor_defs.h>   // our system's unique hardware defines

// This define is a system wide place where we define I2C sem locks and devices
#include <ros_bits/i2c_common_defs.h>

// Our custom ROS Message structure defines
#include <ros_bits/ProxMultiSensorMsg.h>


// Some very basic high level defines specific to this node
#define  SENSOR_POLL_FREQUENCY    1

// Define a semaphore lock for I2C that will be initialized in main and called from callback
// We pull in semaphore code inline rather than a lib as it is very minimal code
#include <ros_bits/ipc_sems.cpp>         // We basically inline the sem calls

// Setup process global semaphore id for I2C lock and ioctl structs for using the sem
int g_i2c_semaphore_id;

// Have a flag to enable hardware monitoring (useful to minimize bus traffic)
int g_enable_sensor_monitoring = 1;

// Pull in inline code for I2C routines. Requires ipc_sems
#include <ros_bits/i2c_utils.cpp>

// Open serial dev and set baud rate
// requires POSIX speed_t baudrates found in termios.h
// Returns non negative for file descriptor or negative for error case
//
int openSerialPortSetBaud(std::string spPortName, speed_t baudRate) {

  int spFd;  // open handle for the port

  spFd = open(spPortName.c_str(), (O_RDWR | O_NONBLOCK | O_NDELAY));  // | O_NOCTTY);
  if (spFd == -1) {
    ROS_ERROR("%s: Error in open of serial port dev '%s'! ", THIS_NODE_NAME, spPortName.c_str());
    return spFd;
  }

  try {

    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( spFd, &tty ) != 0 ) {
        ROS_ERROR("%s: Error in getting port attributes for serial port dev '%s'! ", THIS_NODE_NAME, spPortName.c_str());
        return -2;
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, baudRate);
    cfsetispeed (&tty, baudRate);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  0;                  // read blocks
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( spFd, TCIFLUSH );
    if ( tcsetattr ( spFd, TCSANOW, &tty ) != 0) {
      ROS_ERROR("%s: Error in setting of port attributes for serial port '%s'! ",
        THIS_NODE_NAME, spPortName.c_str());
    }

    usleep(10000);
  } catch (...) {
    if (spFd != -1) {
      close(spFd);
    }
    spFd = -3;
  }
}

// A very simple open of a port but does not set baud rate
int openSerialPortNoBaud(std::string spPortName) {
  int   spFd;                                   // File descriptor for serial port
  spFd = open(spPortName.c_str(), O_RDWR | O_NOCTTY);   // open port for read/write but not as controlling terminal
  if (spFd < 0) {
    ROS_ERROR("%s: Error in open of serial port '%s' for proximity sensor! ", THIS_NODE_NAME, spPortName.c_str());
    // we just return the bad file descriptor also as an error
  }
  return spFd;
}

// Utility to split up string on a delimiter
std::vector<string>  splitString(const std::string &s, char delimChar) {
    vector<string> elements;

    stringstream ss(s);
    string oneElement;
    try {   // Catch any bad operations with this trap
        while (getline(ss, oneElement, delimChar)) {
            elements.push_back(oneElement);
        }
    } catch (...) {  }
    return elements;
}

// Simple transmit with check for correct length.  Nutin fancy 
int sendSerial(int spFd, std::string txString)
{
  char outBuf[128];
  int  txByteCount;
  int  bytesSent;

  if (txString.length() > 127) {
    return -1;    // not very informative but prevents core dump
  }

  txByteCount = txString.length();
  bytesSent = write(spFd, txString.c_str(), txByteCount);    // Write all  bytes to the uart and thus the controller
  if (bytesSent != txByteCount) {   // Write all  bytes to the uart and thus the controller
    ROS_ERROR("%s: Only wrote %d bytes out of %d for firmware rev query!",
        THIS_NODE_NAME, bytesSent, txByteCount);
    return -2;
  }
  return 0;
}

// Read max of len bytes into buffer until a termination char or timeout while trying
// Return number of bytes that were read
int readSerialUntilChar(int spFd, std::string &replyString, char termChar, int len, int timeoutMsec) {

  int      i;
  uint8_t  *inPtr;
  uint8_t  inByte[4];
  char     inBuf[512];
  int      maxChars = 255;
  int      bytes = 0;
  int      byteRetries;

  inPtr = (uint8_t*)inBuf;
  inBuf[0] = 0;		// Just in case termination

  byteRetries = timeoutMsec*2;
  do {
    if (1 == read(spFd, &inByte[0], 1)) {
       bytes += 1;
       if (bytes >= maxChars) {
         return -9;   // Really should 'never' happen but must protect one self ...
       }
       // printf("DEBUG: Read got byte 0x%x \n", inByte);
       *inPtr++ = inByte[0];    // Stash another received char
       byteRetries = 2;   // Reset retries for per byte
       if (inByte[0] == termChar) {   // At termination char
         *inPtr = 0;
		 break;
       }
    } else {
       byteRetries -= 1;
       usleep(500);
    }
  } while ((bytes < len) && (byteRetries > 0));

  //ROS_INFO("%s: Read '%s'  [0x%x, 0x%x, 0x%x, 0x%x ...]",
  //  THIS_NODE_NAME, &inBuf[0], inBuf[0], inBuf[1], inBuf[2], inBuf[3]);

  // Setup user buffer for successful reply
  // const char* cptr = &inBuf[0];
  replyString = inBuf; // cptr;

  return 0;
}

/*
 * initSensorAndInfo()   Setup the sensor for our usage
 *
 * We have to do I2C bus locking external to the ported code that reads the devices.
 *
 * Negative return values mean some sort of error happened
 */
int initSensorAndInfo(int spFd)
{
  int retCode = 0;

  ROS_INFO("%s: Initialize proximity sensor module for 8 sensors. \n", THIS_NODE_NAME);
  retCode = sendSerial(spFd, "fv\r");
  if (retCode != 0) return -1;

  // Setup for 8 sensors to be read
  retCode = sendSerial(spFd, "s8\r");
  if (retCode != 0) return -2;

  retCode = sendSerial(spFd, "a0\r");
  if (retCode != 0) return -2;

  usleep(100000);
 
  return 0;
}

/*
 * getProximitySensorInfo()   Get the latest readings from the sensor module
 *
 * non-zero return values mean some sort of error happened
 */
int getProximitySensorInfo(int spFd, int32_t *proxSensorRanges)
{
  uint8_t statusRegA;
  int     retCode = 0;
  std::string replyFromSensor;
  char    *inPtr;

  if (proxSensorRanges == NULL) {
    return -1;   // bad pointer
  }

  if (g_enable_sensor_monitoring == 0) {
    ROS_DEBUG("%s: Monitoring of sensors is disabled\n", THIS_NODE_NAME);
    return 0;
  }
  
  // Get the real data from the sensors
  retCode = sendSerial(spFd, "qa\r");
  if (retCode != 0) return -2;
 
  retCode = readSerialUntilChar(spFd, replyFromSensor, '>', 250, 500);
  if (retCode != 0) {
    ROS_ERROR("%s: Query from proximity sensor failed with err %d\n", THIS_NODE_NAME, retCode);
  }
  ROS_DEBUG("%s: DEBUG: Query from proximity sensor: %s\n", THIS_NODE_NAME, replyFromSensor.c_str());

  vector<string>  pieces;
  pieces = splitString(replyFromSensor, ':');
  if (pieces.size() != 3) {
    ROS_ERROR("%s: Bad proximity sensor reply format!", THIS_NODE_NAME);
    return -10;
  }
  vector<string>  readings;
  readings = splitString(pieces[1], ',');
  if (readings.size() != 8) {
    ROS_ERROR("%s: Bad proximity sensor reply format! There were %d readings",
      THIS_NODE_NAME, readings.size());
    return -11;
  }

  int range;
  for (int i=0; i<readings.size() ; i++) {
    range = atoi(readings[i].c_str());
    if ((range < 0) || (range > PROX_SENSOR_MAX_RANGE_MM)) {
      proxSensorRanges[i+1] = PROX_SENSOR_BAD_RANGE_DEFAULT;
    } else {
      proxSensorRanges[i+1] = range;    // Seems reasonable so use this range
    }
  }

  // for (int i=0; i<MAX_PROX_SENSORS ; i++) { proxSensorRanges[i+1] = 300 + i; }  // FAKE DATA

  return retCode;
}


//
//  The main: loop periodically polls the sensors and then publishes the readings on ROS topic
//
int main(int argc, char **argv)
{
  int32_t proxRanges[MAX_PROX_SENSORS+1];
  int32_t sensor_count = MAX_PROX_SENSORS;
  std::string sensorSerialDev = std::string(PROX_MULTI_SENSOR_DEV);
  speed_t   baudRate = PROX_MULTI_SENSOR_BAUD;
  char  strBuf[32];
  char  replyBuf[256];
  int retCode;

  // We will setup defaults for the sensor and maybe someday support them from rosparam
  uint8_t radiation_type = PROX_SENSOR_RADIATION_TYPE;
  float   field_of_view  = PROX_SENSOR_FIELD_OF_VIEW_RAD;
  float   min_range      = PROX_SENSOR_MIN_RANGE_METERS;
  float   max_range      = PROX_SENSOR_MAX_RANGE_METERS;

  ros::Publisher proxSensorPublishers[MAX_PROX_SENSORS+1];

  printf("ROS Node starting ros_bits:%s \n", THIS_NODE_NAME);

  // The ros::init() function initializes ROS and needs to see argc and argv
  ros::init(argc, argv, THIS_NODE_NAME);

  // Setup a NodeHandle for the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  // Setup to advertise that we will publish our own system unique message
  ros::Publisher prox_multi_sensor_pub = nh.advertise<ros_bits::ProxMultiSensorMsg>(TOPIC_PROX_MULTI_SENSOR_DATA, 1000);

  ros::Rate loop_rate(SENSOR_POLL_FREQUENCY);

  int paramInt;
  if (nh.getParam("/prox_multi_sensor/sensor_count", paramInt)) {
      sensor_count = paramInt;     // If we found it in ROS parameter server use this value
      if (sensor_count > MAX_PROX_SENSORS) {
        sensor_count = MAX_PROX_SENSORS;
      }
  }
  std::string tmpStr;
  if (nh.getParam("/prox_multi_sensor/port", tmpStr)) {
      sensorSerialDev = tmpStr;     // If we found it in ROS parameter server use this value
  } else {
	sensorSerialDev = PROX_MULTI_SENSOR_DEV;
  }
  ROS_INFO("%s: Using serial device '%s' and %d sensors", THIS_NODE_NAME, sensorSerialDev.c_str(), sensor_count);

   // Now open serial port to get ready to update display
  // int spFd = openSerialPortSetBaud(sensorSerialDev, baudRate);
  int spFd = openSerialPortNoBaud(sensorSerialDev);
  if (spFd < 0) {
    ROS_ERROR("%s: Cannot open serial port '%s'for proximity sensor access! ",
		THIS_NODE_NAME, sensorSerialDev.c_str() );
    return -1;
  }
  ROS_INFO("%s: Serial device initialized. Initialize Sensor Subsystem next", THIS_NODE_NAME);

  // Initialize the proximity sensor module (if required)
  initSensorAndInfo(spFd);

  for (int i=1; i <= sensor_count ; i++) {
    proxRanges[i] = 20 + i;
    sprintf(&strBuf[0],"%d",i);		// There is no itoa() nor is std::to_string() to be found.  Nuts.
    std::string pubTopic = "sonar" + std::string(strBuf);

    // Setup a ROS publisher for each one
    proxSensorPublishers[i] = nh.advertise<sensor_msgs::Range>(pubTopic, 1000);
  }

  ROS_INFO("%s: Sensor Subsystem initialized.", THIS_NODE_NAME);

  usleep(100000);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int msgCount = 0;
  int loopCount = 1;

  while (ros::ok())
  {
   	ROS_DEBUG("%s: DEBUG: Start loop %d ", THIS_NODE_NAME, loopCount);

    // Pick up any changes to ROS parameters
    // int rosParamInt;
    // if (nh.getParam("/imu_lsm303/enable_sensor_monitoring", rosParamInt)) {
    //   g_enable_sensor_monitoring = rosParamInt;
    // }
    g_enable_sensor_monitoring = 1;

    // Read the sensor data now. This higher level routine returns
    // both magnetic and acceleration in X,Y,Z axis order. 
    retCode = getProximitySensorInfo(spFd, &proxRanges[0]);
	if (retCode != 0) {
    	ROS_ERROR("%s: Error in fetch of proximity sensor data!", THIS_NODE_NAME);
	}
   	ROS_DEBUG("%s: Sensor data: %5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d", THIS_NODE_NAME,
      proxRanges[1],proxRanges[2],proxRanges[3],proxRanges[4],proxRanges[5],proxRanges[6],proxRanges[7],proxRanges[8]);

    /*
     * Act on the sensor data by publishing to any of our topic listeners
     */
    sensor_msgs::Range msgRange;
    char frameIdStr[16];
    for (int i=1; i <= sensor_count ; i++) {
      int  rangeDebugOffset = ((loopCount % 20) + i)*5;
      // Publish ros Range message with one per topic for each sensor
      sprintf(frameIdStr,"sensor%d",i);
      msgRange.header.frame_id = std::string(frameIdStr); 
      msgRange.radiation_type  = radiation_type;
      msgRange.field_of_view   = field_of_view;
      msgRange.min_range       = min_range;
      msgRange.max_range       = max_range;
      msgRange.range           = (float)(proxRanges[i] + rangeDebugOffset) * (float)(0.001);
      proxSensorPublishers[i].publish(msgRange);
    }

    // Publish the collision detect info using the CollisionInfo custom message
    ros_bits::ProxMultiSensorMsg msgProxMultiSensor;
    msgProxMultiSensor.sensor1rangeMm = proxRanges[1];
    msgProxMultiSensor.sensor2rangeMm = proxRanges[2];
    msgProxMultiSensor.sensor3rangeMm = proxRanges[3];
    msgProxMultiSensor.sensor4rangeMm = proxRanges[4];
    msgProxMultiSensor.sensor5rangeMm = proxRanges[5];
    msgProxMultiSensor.sensor6rangeMm = proxRanges[6];
    msgProxMultiSensor.sensor7rangeMm = proxRanges[7];
    msgProxMultiSensor.sensor8rangeMm = proxRanges[8];
    std::stringstream ss;
    ss << THIS_NODE_NAME << " publishing [" << msgCount << "]  Proximity multi-sensor data";
    msgProxMultiSensor.comment = ss.str();

    // Publish most recent sensor info on a ROS topic
    prox_multi_sensor_pub.publish(msgProxMultiSensor);

    ++msgCount;

    ros::spinOnce();

    loop_rate.sleep();

    ++loopCount;
  }

  return 0;
}
