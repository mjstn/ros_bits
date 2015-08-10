/*
 * Display Output Subsystem
 *
 * This module implements a ROS node (process) that does Display Updates to an LCD display
 * Accept messages from a ROS Topic (message queue) and update the display.
 * Full update or substring updates starting at a given line and row are supported.
 *
 * Author:    Mark Johnston
 * Creation:  20150608    Initial creation of this module from Mark-Toys.com codebase
 *
 */

#define  THIS_NODE_NAME    "lcd_modtronix"        // The Name for this ROS node

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
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>


#include <ros_bits/lcd_modtronix_defs.h>          // Display Specific defines

#include <ros_bits/i2c_common_defs.h>             // I2C shared hardware bus defines

#include <ros_bits/ipc_sems.cpp>         // We basically inline the sem calls

#include <ros_bits/LcdModtronixMsg.h>    // Message defines


/*
 * Update display hardware specific to our platform
 *
 * As of 7-2014 we output to an I2C Modtronixs serial display
 * 
 * Set row and column to non-zero to position cursor prior to text output
 *
 * Return of 0 is ok, negative values are failure cases
 * If the semLock is supplied we lock a semaphore for the update
 */
int  displayUpdate(std::string text, int attributes, int row, int column, int numChars, int semLock)
{
  int  messageLength = text.length();
  bool dbgPrint = false;

  if (numChars > 0) {
    messageLength = numChars;
  }

  // Would be nice to account for non-zero cursor due to cursor control sometime too ...
  if (messageLength > I2C_LCD_DISPLAY_MAX_CHARS) {
    printf("displayOutput: too many characters to print! \n");
    return -9;
  }

  // Now send data to I2C but under lock and key for just our bits

  if (semLock >= 0) {
    if (ipc_sem_lock(semLock) < 0) { 
      printf("displayOutput: Cannot get sem lock with id %d \n", semLock);
      return -1; 
    }
  }

  int fd;                                       // File descrition
  const char *fileName = I2C_DEV_FOR_LCD_DISPLAY;  // Name of the port we will be using
  int  address = I2C_LCD_DISPLAY_REG_ADDR;   // Address of the Modtronixs LCD display
  unsigned char buf[100];                       // Buffer for data being read/ written on the i2c bus
  char          charBuf[100];                   // Buffer for characters being written on the i2c bus

  if ((fd = open(fileName, O_RDWR)) < 0) {      // Open port for reading and writing
    if (semLock >= 0) {ipc_sem_unlock(semLock);}
    printf("displayOutput: Unable to get bus access to talk to slave\n");
    return -2;
  }

  if (ioctl(fd, I2C_SLAVE, address) < 0) {      // Set the port options and addr of the dev
    printf("displayOutput: Unable to get bus access to talk to slave\n");
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -3;
  }

  // we are now free to transmit to the I2C port 1 to latch 1

  /*
   * Deal with special control modes we see in attributes field
   */
  if ((attributes & DISPLAY_ATTR_CLEAR_FIRST) != 0) {
    // first send the display clear command if requested
    buf[0] = LCD3S_CMD_CLEAR_DISPLAY;
    if ((write(fd, buf, 1)) != 1) {                       // Write commands to the i2c port
      printf("displayOutput: Error writing to i2c slave\n");
      if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
      return -4;
    }
    // TODO: Figure out how to define this msleep(200);
  }

  if ((row > 0) && (column > 0)) {
    // first send the cursor position using input value
    buf[0] = LCD3S_CMD_SET_CURSOR_POSITION;
    buf[1] = (unsigned char)(row);
    buf[2] = (unsigned char)(column);
    if ((write(fd, buf, 3)) != 3) {                       // Write commands to the i2c port
      printf("displayOutput: Error writing to i2c slave\n");
      if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
      return -4;
    }
  }

  // Now send the text part of the message.
  charBuf[0] = LCD3S_CMD_SET_TEXT_AT_CURSOR;
  strncpy(&charBuf[1], text.c_str(), messageLength);

  if (dbgPrint) printf("displayOutput: Now send %d chars to display \n", messageLength);
  if ((write(fd, charBuf, (messageLength+1))) != (messageLength+1)) {    // Write string to display
    printf("displayOutput: Error writing to i2c slave\n");
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -4;
  }

  // next we unlock I2C hardware lock\n", THIS_NODE_NAME);
  if (semLock >= 0) {
    if (ipc_sem_unlock(semLock) < 0) {
      printf("displayOutput: Cannot release I2C lock!  ERROR: %s\n", strerror(errno));
      close (fd);
      return -5;
    }
  }

  close (fd);
  return 0;
}

// Driver to set the power on default message for the display
int displaySetStartupString(int line, std::string text, int semLock)
{
  std::string module = "displaySetStartupString";
  int  messageLength = text.length();
  bool dbgPrint = false;

  if ((line <  1) || (line > 2)) {
    printf("%s: Illegal line number of %d! \n", module.c_str(), line);
    return -9;
  }
  if (messageLength > I2C_LCD_DISPLAY_MAX_CHARS) {
    printf("d%s: too many characters to set for bootup message! \n", module.c_str());
    return -9;
  }

  // Now send data to I2C but under lock and key for just our bits

  if (semLock >= 0) {
    if (ipc_sem_lock(semLock) < 0) { 
      printf("%s: Cannot get sem lock with id %d \n", module.c_str(), semLock);
      return -1; 
    }
  }

  int fd;                                       // File descrition
  const char *fileName = I2C_DEV_FOR_LCD_DISPLAY;  // The I2C port
  int  address = I2C_LCD_DISPLAY_REG_ADDR;   // Address of the Modtronixs LCD display
  unsigned char buf[100];                       // Buffer for data being read/ written on the i2c bus
  char          charBuf[100];                   // Buffer for characters being written on the i2c bus

  if ((fd = open(fileName, O_RDWR)) < 0) {      // Open port for reading and writing
    if (semLock >= 0) {ipc_sem_unlock(semLock);}
    printf("%s: Unable to get bus access to talk to slave\n", module.c_str());
    return -2;
  }

  if (ioctl(fd, I2C_SLAVE, address) < 0) {      // Set the port options and addr of the dev
    printf("%s: Unable to get bus access to talk to slave\n", module.c_str());
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -3;
  }

  // we are now free to transmit to the I2C port 1 to latch 1

  // Now send the text part of the message.
  charBuf[0] = LCD3S_CMD_SET_STARTUP_STRING;
  charBuf[1] = line;
  strncpy(&charBuf[2], text.c_str(), messageLength);
  charBuf[messageLength+3] = 0;

  if (dbgPrint) printf("%s: Now send %d chars to display: '%s' \n", module.c_str(), messageLength, &charBuf[1]);
  if ((write(fd, charBuf, (messageLength+2))) != (messageLength+2)) {    // Write string to display
    printf("%s: Error writing to i2c slave\n", module.c_str());
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -4;
  }

  // next we unlock I2C hardware lock\n", THIS_NODE_NAME);
  if (semLock >= 0) {
    if (ipc_sem_unlock(semLock) < 0) {
      printf("%s: Cannot release I2C lock!  ERROR: %s\n", module.c_str(), strerror(errno));
      close (fd);
      return -5;
    }
  }

  close (fd);
  return 0;
}


// Driver to set the LCD backlight brightness
int displaySetBrightness(int brightness, int semLock)
{
  std::string module = "displaySetBrightness";
  bool dbgPrint = false;

  if ((brightness <  0) || (brightness > 255)) {
    printf("%s: Illegal brightness value of %d! \n", module.c_str(), brightness);
    return -9;
  }

  // Now send command to I2C but under lock and key for just our bits

  if (semLock >= 0) {
    if (ipc_sem_lock(semLock) < 0) { 
      printf("%s: Cannot get sem lock with id %d \n", module.c_str(), semLock);
      return -1; 
    }
  }

  int fd;                                       // File descrition
  const char *fileName = I2C_DEV_FOR_LCD_DISPLAY;  // Name of the port we will be using
  int  address = I2C_LCD_DISPLAY_REG_ADDR;   // Address of the Modtronixs LCD display
  unsigned char buf[100];                       // Buffer for data being read/ written on the i2c bus
  char     charBuf[100];                        // Buffer for characters being written on the i2c bus

  if ((fd = open(fileName, O_RDWR)) < 0) {      // Open port for reading and writing
    if (semLock >= 0) {ipc_sem_unlock(semLock);}
    printf("%s: Unable to get bus access to talk to slave\n", module.c_str());
    return -2;
  }

  if (ioctl(fd, I2C_SLAVE, address) < 0) {      // Set the port options and addr of the dev
    printf("%s: Unable to get bus access to talk to slave\n", module.c_str());
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -3;
  }

  // we are now free to transmit to the I2C port 1 to latch 1

  // Now send the command
  charBuf[0] = LCD3S_CMD_BACKLIGHT_LEVEL;
  charBuf[1] = brightness;

  if (dbgPrint) printf("displaySetBrightness: Now send command to set brightness to %d \n", brightness);
  if ((write(fd, charBuf, 2)) != 2) {    // Write did not write all bytes
    printf("%s: Error writing to i2c slave\n", module.c_str());
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    return -4;
  }

  // next we unlock I2C hardware lock\n", THIS_NODE_NAME);
  if (semLock >= 0) {
    if (ipc_sem_unlock(semLock) < 0) {
      printf("%s: Cannot release I2C lock!  ERROR: %s\n", module.c_str(), strerror(errno));
      close (fd);
      return -5;
    }
  }

  close (fd);
  return 0;
}


/**
 * Receive messages for display output
 */
void displayApiCallback(const ros_bits::LcdModtronixMsg::ConstPtr& msg)
{
  ROS_DEBUG("%s heard display output msg: of actionType %d row %d column %d numChars %d attr 0x%x text %s comment %s]",
                THIS_NODE_NAME, msg->actionType, msg->row, msg->column, msg->numChars, msg->attributes, 
                msg->text.c_str(), msg->comment.c_str());

   // Now send data to the display
   int i2cSemLockId = -9;
   if (SEM_PROTECT_I2C == 0) {
     i2cSemLockId = IPC_SEM_DUMMY_SEMID;
     ROS_INFO("%s: I2C sem DUMMY lock used so user code can run \n", THIS_NODE_NAME);
   } else {
     i2cSemLockId = ipc_sem_get_by_key_with_wait(I2C_LCD_DISPLAY_SEM_KEY, 10);
     if (i2cSemLockId < 0) {
       ROS_INFO("%s: Cannot acquire lock for I2C! Skip Message! ", THIS_NODE_NAME);
       return;
     }
   }

   switch (msg->actionType) {
     case MSG_DISPLAY_STARTUP_STRING:
       displaySetStartupString(msg->row, msg->text.c_str(), i2cSemLockId);
       break;
     case MSG_DISPLAY_SET_BRIGHTNESS:
       displaySetBrightness(msg->attributes, i2cSemLockId);
       break;
     case MSG_DISPLAY_ALL:
     case MSG_DISPLAY_SUBSTRING:
       displayUpdate(msg->text.c_str(), msg->attributes, msg->row, msg->column, msg->numChars, i2cSemLockId);
       break;
     default:
       break;
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
  // In this system our main node initializes this semaphore if SEM_PROTECT_I2C is not zero
  int i2cSemLockId = -9;
  if (SEM_PROTECT_I2C == 0) {
    i2cSemLockId = IPC_SEM_DUMMY_SEMID;
    ROS_INFO("%s: I2C sem DUMMY lock used so user code can run \n", THIS_NODE_NAME);
  } else {
    /**
    * Setup I2C semaphore locks for our system and other nodes must wait till this happens
    */
    ROS_INFO("%s: Setup I2C sem lock for the system or die.  \n", THIS_NODE_NAME);
    i2cSemLockId = ipc_sem_init_lock(I2C_LCD_DISPLAY_SEM_KEY);
    if (i2cSemLockId < 0) {
      ROS_ERROR("%s: Cannot initialize I2C lock for key %d! \n", THIS_NODE_NAME, I2C_LCD_DISPLAY_SEM_KEY);
      exit; // This is system wide fatal so stop here perhaps, other nodes will abort.
    }
    ROS_INFO("%s: The system's I2C sem lock has been setup with sem ID %d.\n", THIS_NODE_NAME, i2cSemLockId);
  }

  ROS_INFO("%s: Display subsystem ready! ", THIS_NODE_NAME);

  //  You can setup a message on the display to get going prior to main code setting a message
  //  displayUpdate("    Display         starting  ", 0, 1, 1, 0, i2cSemLockId);

  // Set to subscribe to the display topic and we then get callbacks for each message
  ros::Subscriber sub = nh.subscribe(TOPIC_LCD_DISPLAY_OUTPUT, 1000, displayApiCallback);

  // ros::spin() will enter a loop allowing callbacks to happen. Exits on Ctrl-C
  ros::spin();

  return 0;
}
