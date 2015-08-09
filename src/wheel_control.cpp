/*
 * Wheel Control Subsystem
 *
 * This module implements a ROS node (process) that manages 2 wheels used for the system
 * 
 * In this model it is assumed there will be a main node that takes in cmd_vel and makes decisions
 * as to how to drive the motor control in this node using messages unique to this subsystem
 *
 * We could choose to have this node also take in a twist message perhaps
 * with the name of  /cmd_vel_motor  but we do not do that at this time
 *
 * This node was written to control a Mark-Toys motor control signal generating board
 * where the motor control takes the form to generate PWM and the 2-bit control signals
 * as taken in by the Monster Moto Shield dual VNH2SP30 motor controller
 *
 * Author:    Mark Johnston
 * Creation:  20150807    Initial creation of this module from Mark-Toys fiddlerbot robot wheel_control
 *
 */

#define  THIS_NODE_NAME    "wheel_control"      // The Name for this ROS node

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
#include <ros_bits/wheel_control_defs.h>       // Wheel control message element defines

// Our custom ROS Message structure defines
#include <ros_bits/WheelControlMsg.h>


// Define a semaphore lock for I2C that will be initialized in main and called from callback
// We pull in semaphore code inline rather than a lib as it is very minimal code
#include <ros_bits/i2c_common_defs.h>             // I2C shared hardware bus defines
#include <ros_bits/ipc_sems.cpp>         // We basically inline the sem calls
int g_i2c_semaphore_id;

// Set some global process space locations for wheel hardware info
int g_right_wheel_speed = 0;
int g_right_wheel_control;
int g_left_wheel_speed = 0;
int g_left_wheel_control;

/*
 * Converts speed value to PWM control value.  This only accepts positive speed, direction is not part of this
 *
 * We take in speed 0 to 100 percent and convert it to PWM value
 * all out of range values are treated as 0 or off.
 *
 * If we wish to calibrate speed to wheel drive for each wheel it should be done here
 */
int speedToPwmPercent(int motorNumber, int motSpeed) {

   int speed_right  = 0;
   int speed_left   = 0;
   int speed  = 0;

   speed = motSpeed;
   if ((speed < 0) || (speed > 100)) {
     speed = 0;  // Illegal values to to off
     return -1;
   }

   // HERE is where we could calibrate depending on the wheel number
   // 
   speed_right = speed_left = speed;  // Right now we do no correction so just set to speed
 
   switch (motorNumber) {
   case MSG_MOTOR_RIGHT_NUMBER:
     speed  = speed_right ;
     break;
   case MSG_MOTOR_LEFT_NUMBER:
     speed  = speed_left ;
     break;
   default:
     speed  = 0;
     break;
   }

   return speed ;
}



// Routine to convert inbound message wheel control to hardware specific modes
int motMsgCtrlToDrvCtrl(int motDrvPwmPercent, int drvMsgCtrl) {
  int motDrvControl;

  if (motDrvPwmPercent == 0) {
    motDrvControl = MT_MOT_CTRL_BREAK_TO_GND;
  } else {
    switch (drvMsgCtrl) {
      case MSG_MOTOR_DIRECTION_FORWARD:
        motDrvControl = MT_MOT_CTRL_DIR_CLOCKWISE;
        break;
      case MSG_MOTOR_DIRECTION_REVERSE:
        motDrvControl = MT_MOT_CTRL_DIR_CTR_CLOCKWISE;
        break;
      case MSG_MOTOR_BREAK_VCC:
        motDrvControl = MT_MOT_CTRL_BREAK_TO_VCC;
        break;
      default:
      case MSG_MOTOR_BREAK_GND:
        motDrvControl = MT_MOT_CTRL_BREAK_TO_GND;
        break;
    }
  }

  return motDrvControl;
}

   
/*
 * motorDriver()  Drives both wheels on one call.
 *
 * This call sets wheel driver PWM and control bits for the Mark-Toys wheel control board
 * to control the board over I2C interface
 *
 * This routine tries to do the I2C using semaphore lock IF the semId is >= 0
 *
 * Negative return values mean some sort of error happened
 */
int motorDriver(int rightWheelPWMPercent, int rightWheelControl,
                int leftWheelPWMPercent,  int leftWheelControl, int semLock) 
{
  int i2cAddress;
  int motDrvControl;
  int motDrvPwmPercent;

  // To send data over I2C we must aquire a OS lock
  if (semLock >= 0) {
    if (ipc_sem_lock(semLock) < 0) { return -1; }
  }

  int fd;					// File descrition
  unsigned char buf[10];			// Buffer for data being read/ written on the i2c bus
  
  const char *i2cDevName = I2C_DEV_FOR_WHEEL_CONTROL;	// Name of the port we will be using
	
  if ((fd = open(i2cDevName, O_RDWR)) < 0) {	// Open port for reading and writing
    if (semLock >= 0) {ipc_sem_unlock(semLock);}
    return -2;
  }

  // Drive the right wheel motor
  motDrvPwmPercent = rightWheelPWMPercent;
  motDrvControl = motMsgCtrlToDrvCtrl(motDrvPwmPercent, rightWheelControl);

  i2cAddress = I2C_WHEEL_CONTROL_REG_ADDR;		// Address of motor controller on I2C
  if (ioctl(fd, I2C_SLAVE, i2cAddress) < 0) {	// Set the port options and addr of the dev 
    printf("Unable to get bus access to talk to slave motor controller over I2C\n");
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -3;
  }

  buf[0] = MT_MOT_CTRL_CMD_SET_MOTOR;	// Command byte to the controller
  buf[1] = MT_MOT_CTRL_RIGHT_MOTOR;
  buf[2] = motDrvPwmPercent;
  buf[3] = motDrvControl;
  ROS_DEBUG("%s: motorDriver Right Wheel Write Bytes: Addr 0x%x channel %d PwmPct %d Control %d ",
                THIS_NODE_NAME, i2cAddress, buf[1], buf[2], buf[3]);
  if ((write(fd, buf, 4)) != 4) {			// Write commands to the i2c port
    printf("Error writing to i2c slave motor controller\n");
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -4;
  }


  // Drive the right wheel motor
  motDrvPwmPercent = leftWheelPWMPercent;
  motDrvControl = motMsgCtrlToDrvCtrl(motDrvPwmPercent, leftWheelControl);

  i2cAddress = I2C_WHEEL_CONTROL_REG_ADDR;		// Address of motor controller on I2C
  if (ioctl(fd, I2C_SLAVE, i2cAddress) < 0) {	// Set the port options and addr of the dev 
    printf("Unable to get bus access to talk to slave motor controller over I2C\n");
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -3;
  }

  buf[0] = MT_MOT_CTRL_CMD_SET_MOTOR;	// Command byte to the controller
  buf[1] = MT_MOT_CTRL_LEFT_MOTOR;
  buf[2] = motDrvPwmPercent;
  buf[3] = motDrvControl;
  ROS_DEBUG("%s: motorDriver Left Wheel Write Bytes: Addr 0x%x channel %d PwmPct %d Control %d ",
                THIS_NODE_NAME, i2cAddress, buf[1], buf[2], buf[3]);
  if ((write(fd, buf, 4)) != 4) {			// Write commands to the i2c port
    printf("Error writing to i2c slave motor controller\n");
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -4;
  }

  close (fd);         // Close the I2C handle

  if (semLock >= 0) {
    if (ipc_sem_unlock(semLock) < 0) { 
      // printf("Cannot release I2C lock!  ERROR: %s\n", strerror(errno));
      close (fd);
      return -9; 
    }
  }

  return 0;
}


/**
 * Receive mmessages for serial api over the ROS system.
 */
void wheelControlCallback(const ros_bits::WheelControlMsg::ConstPtr& msg)
{
   int right_wheel_speed = 0;
   int left_wheel_speed = 0;
   int speed_right  = 0;
   int speed_left   = 0;

   int right_wheel_control = msg->control;    
   int left_wheel_control  = msg->control;    

   bool driveMotors = true;

   // Here we convert speeds of 0-9 to signed dac bits
   // We must extract ADC speed bits for the DAC from direction which is the sign before use.
   //
   speed_right  = speedToPwmPercent(MSG_MOTOR_RIGHT_NUMBER, msg->speed);
   speed_left   = speedToPwmPercent(MSG_MOTOR_LEFT_NUMBER,  msg->speed);


   // Here we decode the inbound commad and set motor speeds and directions
   switch (msg->actionType) {
     case MSG_MOTOR_CONTROL_STOP:
       g_right_wheel_speed = 0;
       g_left_wheel_speed  = 0;
       break;

     case MSG_MOTOR_CONTROL_RIGHT:
       g_right_wheel_speed  = speed_right ;
       g_right_wheel_control = right_wheel_control;
       break;

     case MSG_MOTOR_CONTROL_LEFT:
       g_left_wheel_speed  = speed_left ;
       g_left_wheel_control = left_wheel_control;
       break;

     case MSG_MOTOR_CONTROL_BOTH:
       g_right_wheel_speed  = speed_right ;
       g_right_wheel_control = right_wheel_control;
       g_left_wheel_speed   = speed_left ;
       g_left_wheel_control  = left_wheel_control;
       break;

     default:
       driveMotors = false;
       break;
   } 

   // Drive the dual-dacs now using separate routine specific to the DACs
   ROS_INFO("%s: motor DACs Right:0x%x Left=0x%x  msg: actionType %d motorNumber %d CtrlBits %d speed %d comment %s]",
                THIS_NODE_NAME, speed_right , speed_left , 
                msg->actionType, msg->motorNumber, msg->control, msg->speed, msg->comment.c_str());

   // Drive the hardware
   int retCode = motorDriver(g_right_wheel_speed, g_right_wheel_control,
               g_left_wheel_speed,  g_left_wheel_control, 
               g_i2c_semaphore_id);
   if (retCode < 0) {
     ROS_ERROR("%s: motorDriver ERROR %d", THIS_NODE_NAME, retCode);
   }

}

int main(int argc, char **argv)
{
   printf("ROS Node starting ros_bits:%s \n", THIS_NODE_NAME);

  // The ros::init() function initializes ROS and needs to see argc and argv
  ros::init(argc, argv, THIS_NODE_NAME);

  // Setup a NodeHandle for the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  // Setup process global semaphore id for I2C lock and ioctl structs for using the sem
  // In this system our main node initializes this semaphore
  int semMaxWaitSec = 300;
  ROS_INFO("%s: Wait for I2C sem lock for up to %d seconds or die.  \n", THIS_NODE_NAME,  semMaxWaitSec);
  g_i2c_semaphore_id = ipc_sem_get_by_key_with_wait(I2C_WHEEL_CONTROL_SEM_KEY, semMaxWaitSec);
  if (g_i2c_semaphore_id < 0) {
    ROS_ERROR("%s: Cannot obtain I2C lock for key %d!  ERROR: %d\n", THIS_NODE_NAME, 
      I2C_WHEEL_CONTROL_SEM_KEY, g_i2c_semaphore_id);
    exit(1);
  }
  ROS_INFO("%s: I2C sem lock obtained as value %d \n", THIS_NODE_NAME,  g_i2c_semaphore_id);

  // Subscribe to messages and as they come in the indicated callback is envolked
  ros::Subscriber sub = nh.subscribe(TOPIC_WHEEL_CONTROL, 1000, wheelControlCallback);

  // Fall into ROS and receive callbacks out of the spin.  Ctrl-C will return from the spin
  ros::spin();

  return 0;
}
