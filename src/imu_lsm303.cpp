/*
 * Node to monitor and publish Acceleration and Compass data from Lsm303 I2C hardware
 *
 * This module implements a ROS node (process) that monitors the compass and accelerometer.
 *
 * Output is in a custom message for raw IMU parameters and also will be output in
 * the ROS standard message of sensor_msgs/Imu.msg
 *
 * Author:    Mark Johnston
 * Creation:  20150605    Initial creation of this module from Mark-Toys.com codebase
 *
 */

#define  THIS_NODE_NAME    "imu_lsm303"   // The Name for this ROS node

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
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

// These includes are very specific to our hardware and system software internals
#include <ros_bits/imu_lsm303_defs.h>   // our system's unique hardware defines

// This define is a system wide place where we define I2C sem locks and devices
#include <ros_bits/i2c_common_defs.h>

// Our custom ROS Message structure defines
#include <ros_bits/ImuLsm303Msg.h>

// The Standard ROS sensor_msgs/Imu.msg format
#include <sensor_msgs/Imu.h>

// Some very basic high level defines specific to this node
#define  IMU_LSM303_POLL_FREQUENCY    1

// Define a semaphore lock for I2C that will be initialized in main and called from callback
// We pull in semaphore code inline rather than a lib as it is very minimal code
#include <ros_bits/ipc_sems.cpp>         // We basically inline the sem calls

// Setup process global semaphore id for I2C lock and ioctl structs for using the sem
int g_i2c_semaphore_id;

// Have a flag to enable hardware monitoring (useful to minimize bus traffic)
int g_enable_sensor_monitoring = 1;

// Pull in inline code for I2C routines. Requires ipc_sems
#include <ros_bits/i2c_utils.cpp>

// Pull in code for LSM303 Compass - Accelerometer board from AdaFruit. Requires i2c_utils
#include <ros_bits/lsm303.h>
#include <ros_bits/vector.h>


// We inline the code for drivers of the LSM303 device
#include <ros_bits/vector.c>
#include <ros_bits/lsm303_drivers.c>


/*
 * initSensorAndInfo()   Setup the magnetic and accellerometer data and driver
 *
 * We have to do I2C bus locking external to the ported code that reads the devices.
 *
 * Negative return values mean some sort of error happened
 */
int initSensorAndInfo(void)
{

  ROS_DEBUG("%s: Initialize magnetic and accellerometer sensors\n", THIS_NODE_NAME);
 
  if (ipc_sem_lock(g_i2c_semaphore_id) < 0) {
      ROS_ERROR("%s: Magnetic and accellerometer sensor configuration sem lock error!\n", THIS_NODE_NAME);
      return -1;
  }

  LSM303_Configuration();

  if (ipc_sem_unlock(g_i2c_semaphore_id) < 0) {
      ROS_ERROR("%s: Magnetic and accellerometer sensor configuration error!\n", THIS_NODE_NAME);
      return -2;
  }

  ROS_DEBUG("%s: Magnetic and accellerometer sensors initialized\n", THIS_NODE_NAME);

  return 0;
}

/*
 * getSensorInfo()   Get the magnetic and accellerometer data   
 *
 * The 3 values for mag and accel come back from THIS routine in X,Y,Z order
 * per the silkscreen on the board.
 *
 * NOTE: In the lower level routines magnetic values are read as X, Z, then Y.
 *
 * We have to do I2C bus locking external to the ported code that reads the devices.
 *
 * non-zero return values mean some sort of error happened
 */
int getSensorInfo(int16_t *magneticData, int16_t *accelData)
{
  uint8_t statusRegA;

  if (g_enable_sensor_monitoring == 0) {
    ROS_DEBUG("%s: Monitoring of sensors is disabled\n", THIS_NODE_NAME);
    return 0;
  }
  
  ROS_INFO("%s: Fetch magnetic sensor data\n", THIS_NODE_NAME);

  if (ipc_sem_lock(g_i2c_semaphore_id) < 0) {
      ROS_ERROR("%s: Magnetic and accellerometer sensor fetch sem lock error!\n", THIS_NODE_NAME);
      return -1;
  }

  // The magnetic values are read as X, Z, then Y per silkscreen on the board
  // so we shift the order to X,Y,Z here
  int16_t swapVal;
  LSM303_Magn_Read_Magn(&magneticData[0], false);
  swapVal = magneticData[1];          // The Z axis from lower level routines
  magneticData[1] = magneticData[2]; 
  magneticData[2] = swapVal;        

  ROS_INFO("%s: Fetch accelerometer sensor data\n", THIS_NODE_NAME);
  // DEBUG:  Read the status byte for accelerometer
  i2c_BufferRead(I2C_DEV_FOR_IMU_LSM303, I2C_IMU_LSM303_SEM_KEY,
      LSM_A_ADDRESS, &statusRegA, LSM_A_STATUS_REG_ADDR, 1); 

  // The 3 acceleration values are for X, Y, then Z per silkscreen on the board
  LSM303_Acc_Read_Acc(&accelData[0], true);

  // Read the status byte for accelerometer to form return code
  i2c_BufferRead(I2C_DEV_FOR_IMU_LSM303, I2C_IMU_LSM303_SEM_KEY,
      LSM_A_ADDRESS, &statusRegA, LSM_A_STATUS_REG_ADDR, 1); 

  if (ipc_sem_unlock(g_i2c_semaphore_id) < 0) {
      ROS_ERROR("%s: Magnetic and accellerometer sensor fetch sem unlock error!\n", THIS_NODE_NAME);
      return -2;
  }

  ROS_INFO("%s: LSM303 sensor data fetched.\n", THIS_NODE_NAME);

  return statusRegA & 0x0f;
}


//
//  The main loop periodically polls the sensors and then publishes the readings on ROS topic
//
int main(int argc, char **argv)
{
   printf("ROS Node starting ros_bits:%s \n", THIS_NODE_NAME);

   // The ros::init() function initializes ROS and needs to see argc and argv
  ros::init(argc, argv, THIS_NODE_NAME);

  // Setup a NodeHandle for the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  // Setup to advertise the standard ROS sensor_msgs/Imu.msg format 
  ros::Publisher ros_imu_pub = nh.advertise<sensor_msgs::Imu>("ros_imu", 1000);

  // Setup to advertise that we will publish our own system unique message
  ros::Publisher imu_lsm303_pub = nh.advertise<ros_bits::ImuLsm303Msg>("imu_lsm303", 1000);

  ros::Rate loop_rate(IMU_LSM303_POLL_FREQUENCY);

  // Setup process global semaphore id for I2C lock and ioctl structs for using the sem
  // In this system our main node initializes this semaphore if SEM_PROTECT_I2C is not zero
  if (SEM_PROTECT_I2C == 0) {
   g_i2c_semaphore_id = IPC_SEM_DUMMY_SEMID;
   ROS_INFO("%s: I2C sem DUMMY lock used so user code can run \n", THIS_NODE_NAME);
  } else {  
    int semMaxWaitSec = 300;
    ROS_INFO("%s: Wait for I2C sem lock for up to %d seconds or die.  \n", THIS_NODE_NAME,  semMaxWaitSec);
    g_i2c_semaphore_id = ipc_sem_get_by_key_with_wait(I2C_IMU_LSM303_SEM_KEY, semMaxWaitSec);
    if (g_i2c_semaphore_id < 0) {
      ROS_ERROR("%s: Cannot obtain I2C lock for key %d!  ERROR: %d\n", THIS_NODE_NAME,
        I2C_IMU_LSM303_SEM_KEY, g_i2c_semaphore_id);
      exit(1);
    }
    ROS_INFO("%s: I2C sem lock obtained as value %d \n", THIS_NODE_NAME,  g_i2c_semaphore_id);
  }

  // Initialize Accellerometer and compass hardware and setup driver context
  // As of 10/2014 the driver we use is non-reentrant for one device per ROS node
  initSensorAndInfo();

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int msgCount = 0;
  int loopCount = 0;

  int16_t magnRawData[16];
  int16_t accelRawData[16];
  int   pitch;
  int   roll;
  int   heading;

  int lastIrSensorInfo = 0;     // This will come from real hardware
  while (ros::ok())
  {

    // Pick up any changes to ROS parameters
    int rosParamInt;
    if (nh.getParam("/imu_lsm303/enable_sensor_monitoring", rosParamInt)) {
      g_enable_sensor_monitoring = rosParamInt;
    }
     
    //  Read sensors and broadcast to subscribers on our topic
    ROS_DEBUG("%s: lock I2c sem with id %d prior to reading sensors ... \n", THIS_NODE_NAME, g_i2c_semaphore_id);

    // Read the sensor data now. This higher level routine returns
    // both magnetic and acceleration in X,Y,Z axis order. 
    getSensorInfo(&magnRawData[0], &accelRawData[0]);

    // Calculate derived values using raw sensor data in X,Y,Z order
    LSM303_CalPitchRollHeading(magnRawData, accelRawData);
    pitch   = LSM303_GetPitch();
    roll    = LSM303_GetRoll();
    heading = LSM303_GetHeading();

    /*
     * Act on the sensor data by publishing to any of our topic listeners
     */

    // Publish the collision detect info using the CollisionInfo custom message
    ros_bits::ImuLsm303Msg msgImuLsm303;
    msgImuLsm303.xAcceleration = accelRawData[0];
    msgImuLsm303.yAcceleration = accelRawData[1];
    msgImuLsm303.zAcceleration = accelRawData[2];
    msgImuLsm303.xMagnetic = magnRawData[0];;
    msgImuLsm303.yMagnetic = magnRawData[1];;
    msgImuLsm303.zMagnetic = magnRawData[2];;
    msgImuLsm303.pitch = pitch;
    msgImuLsm303.roll = roll;
    msgImuLsm303.heading = heading;
    std::stringstream ss;
    ss << THIS_NODE_NAME << " publishing [" << msgCount << "]  Magnetic and Acceleration Data ";
    msgImuLsm303.comment = ss.str();

    // Publish most recent sensor info on a ROS topic
    imu_lsm303_pub.publish(msgImuLsm303);

    ++msgCount;

    ros::spinOnce();

    loop_rate.sleep();

    ++loopCount;
  }

  return 0;
}
