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
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

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


/*
 * initSensorAndInfo()   Setup the sensor for our usage
 *
 * We have to do I2C bus locking external to the ported code that reads the devices.
 *
 * Negative return values mean some sort of error happened
 */
int initSensorAndInfo(void)
{
  ROS_DEBUG("%s: Initialize proximity sensor module. (Just use defaults)\n", THIS_NODE_NAME);
 
  return 0;
}

/*
 * getProximitySensorInfo()   Get the latest readings from the sensor module
 *
 * non-zero return values mean some sort of error happened
 */
int getProximitySensorInfo(std::string serialDev, int32_t *proxSensorRanges)
{
  uint8_t statusRegA;

  if (proxSensorRanges == NULL) {
    return -1;   // bad pointer
  }

  if (g_enable_sensor_monitoring == 0) {
    ROS_DEBUG("%s: Monitoring of sensors is disabled\n", THIS_NODE_NAME);
    return 0;
  }
  
  // TODO: !!! Get the real data from the sensors
  for (int i=0; i<MAX_PROX_SENSORS ; i++) {
    proxSensorRanges[i+1] = 300 + i;
  }

  return 0;
}


//
//  The main: loop periodically polls the sensors and then publishes the readings on ROS topic
//
int main(int argc, char **argv)
{
  int32_t proxSensorRanges[MAX_PROX_SENSORS+1];
  int32_t sensor_count = MAX_PROX_SENSORS;
  std::string sensorSerialDev = std::string(PROX_MULTI_SENSOR_DEV);
  char  strBuf[32];

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
  }
  ROS_INFO("%s: Using serial device '%s' and %d sensors", THIS_NODE_NAME, sensorSerialDev.c_str(), sensor_count);

  // Initialize the proximity sensor module (if required)
  initSensorAndInfo();

  for (int i=1; i <= sensor_count ; i++) {
    proxSensorRanges[i] = 20 + i;
    sprintf(&strBuf[0],"%d",i);		// There is no itoa() nor is std::to_string() to be found.  Nuts.
    std::string pubTopic = "sonar" + std::string(strBuf);

    // Setup a ROS publisher for each one
    proxSensorPublishers[i] = nh.advertise<sensor_msgs::Range>(pubTopic, 1000);
  }

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int msgCount = 0;
  int loopCount = 0;

  while (ros::ok())
  {
    // Pick up any changes to ROS parameters
    // int rosParamInt;
    // if (nh.getParam("/imu_lsm303/enable_sensor_monitoring", rosParamInt)) {
    //   g_enable_sensor_monitoring = rosParamInt;
    // }
    g_enable_sensor_monitoring = 1;
     
    // Read the sensor data now. This higher level routine returns
    // both magnetic and acceleration in X,Y,Z axis order. 
    getProximitySensorInfo(std::string(PROX_MULTI_SENSOR_DEV), &proxSensorRanges[0]);

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
      msgRange.range           = (float)(proxSensorRanges[i] + rangeDebugOffset) * (float)(0.001);
      proxSensorPublishers[i].publish(msgRange);
    }
    ROS_DEBUG("%s: Ranges: %4d %4d %4d %4d %4d %4d %4d %4d ", THIS_NODE_NAME, 
		proxSensorRanges[1], proxSensorRanges[2], proxSensorRanges[3], proxSensorRanges[4], 
		proxSensorRanges[5], proxSensorRanges[6], proxSensorRanges[7], proxSensorRanges[8]); 

    // Publish the collision detect info using the CollisionInfo custom message
    ros_bits::ProxMultiSensorMsg msgProxMultiSensor;
    msgProxMultiSensor.sensor1rangeMm = proxSensorRanges[1];
    msgProxMultiSensor.sensor2rangeMm = proxSensorRanges[2];
    msgProxMultiSensor.sensor3rangeMm = proxSensorRanges[3];
    msgProxMultiSensor.sensor4rangeMm = proxSensorRanges[4];
    msgProxMultiSensor.sensor5rangeMm = proxSensorRanges[5];
    msgProxMultiSensor.sensor6rangeMm = proxSensorRanges[6];
    msgProxMultiSensor.sensor7rangeMm = proxSensorRanges[7];
    msgProxMultiSensor.sensor8rangeMm = proxSensorRanges[8];
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
