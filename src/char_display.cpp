/*
 * Display Output Subsystem
 *
 * This module implements a ROS node (process) that does Display Updates to an LCD display
 * Accept messages from a ROS Topic (message queue) and spool updates to the display.
 * Full update or substring updates starting at a given line and row are supported.
 *
 * This node drives the simple minded Parallax 27977-RT 2x16 serial LCD display
 * 
 * Some effort to allow this node to accept legacy lcd modtronix or newer general message
 * will be made to minimize impact and allow swap in of this display easier.
 *
 * Author:    Mark Johnston
 * Creation:  20150924    Initial creation of this module from Mark-Toys.com lcd_modtronix node
 */

#define  THIS_NODE_NAME          "char_display"        // The Name for this ROS node
#define  NODE_LOOPS_PER_SEC      20

//                                12345678901234567890123456789012
#define  DISP_STARTUP_STRING     "   Display         Starting     "

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
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
// #include <sys/types.h>
// #include <sys/time.h>
// #include <sys/stat.h>
// #include <unistd.h>
#include <iterator>
#include <list>
#include <queue>

using namespace std;

#include <ros_bits/lcd_modtronix_defs.h>          // Modtronics Display Specific defines
#include <ros_bits/lcd_parallax_defs.h>           // Serial Display Specific defines
#include <ros_bits/serial_common_defs.h>          // Serial port info

#include <ros_bits/CharDisplayMsg.h>     // Message defines
#include <ros_bits/LcdModtronixMsg.h>    // Message defines so we can accept legacy I2C topic

// Define a packet so we can spool messages received in callbacks
class DispCmd {
    public:

    int    actionType;
    int    row;
    int    column;
    int    numChars;
    int    attributes;
    std::string text;
    std::string comment;

    // Constructor must supply all items but we have a blank one too

    DispCmd() :
        actionType(0), row(0), column(0), numChars(0), attributes(0), text(""), comment("Null cmd") {}

    DispCmd(int a, int r, int c, int n, int atr, std::string txt, std::string cmt) :
        actionType(a), row(r), column(c), numChars(n), attributes(atr), text(txt), comment(cmt) {}

   // supply a readable name for the goal type
    std::string getCmdName() {
      std::string cmdName;
      switch (this->actionType) {
        case MSG_DISPLAY_ALL:            cmdName = "Write to full display";     break;
        case MSG_DISPLAY_SUBSTRING:      cmdName = "Display substring";         break;
        case MSG_DISPLAY_STARTUP_STRING: cmdName = "Display startup message";   break;
        case MSG_DISPLAY_SET_BRIGHTNESS: cmdName = "Set display brightness";    break;
        default:                         cmdName = "Unknown command";           break;
      }
      return cmdName;
    }
};

// A queue so callbacks can spool to display background task
queue<DispCmd>  g_CmdQueue;

// We maintain an image of what text is on the display
#define DISP_MAX_TEXT      CHAR_DISPLAY_MAX_CHARS;
#define DISP_MAX_COMMENT   128

// Routines to flush out all commands not yet processed
void clearCmdQueue()
{
  DispCmd cmdToClear;
  ROS_INFO("%s: Clearing all pending commands. Flushing pending ueue now ",THIS_NODE_NAME);
  while( !g_CmdQueue.empty() ) {
    cmdToClear = g_CmdQueue.front();
    g_CmdQueue.pop();
    ROS_INFO("%s: Clearing queued Display CMD  '%s' [%d] ",THIS_NODE_NAME,
         cmdToClear.getCmdName().c_str(), cmdToClear.actionType );
  }
}


std::string  g_serialPortName;

// We use the same call open helper call
int openSerialPort(std::string spPortName ) {
  int   spFd;                                   // File descriptor for serial port
  std::string charDispSerialPort(CHAR_DISP_CONTROL_DEVICE);
  spFd = open(spPortName.c_str(), O_RDWR | O_NOCTTY);   // open port for read/write but not as controlling terminal
  if (spFd < 0) {
    ROS_ERROR("%s: Error in open of serial port '%s'! ", THIS_NODE_NAME, spPortName.c_str());
    // we just return the bad file descriptor also as an error
  }
  return spFd;
}

/*
 * Update display hardware specific to our platform
 *
 * This routine updates simple minded Parallax 27977-RT 2x16 serial LCD display
 * 
 * Setting numChars to 0 uses the text string length for the transmit length
 * You may set row and column to non-zero for printing only first few chars.
 * If numChars is non-zero AND < length of string we just print substring
 *
 * WARNING:  Serial port must be set to 9600 baud and switch on unit set to 9600 as well.
 * I am having to insert waits as apparently the driver is not waiting for Rx empty
 *
 * Return of 0 is ok, negative values are failure cases
 * If the semLock is supplied we lock a semaphore for the update
 */
int  displayUpdate(std::string text, int attributes, int row, int column, int numChars)
{
  bool dbgPrint = false;
  static char outBuf[CHAR_DISPLAY_MAX_CHARS+4];
  int bytesSent;

  if (text.length() < numChars) {
    ROS_ERROR("%s: displayUpdate: Request to output %d chars but text is only %d char long!\n",
             THIS_NODE_NAME, numChars, text.length());
  }

  int  messageLength = text.length();
  if (numChars > 0) {
    messageLength = numChars;
  }

  // Would be nice to account for non-zero cursor due to cursor control sometime too ...
  if (messageLength > CHAR_DISPLAY_MAX_CHARS) {
    ROS_ERROR("%s: Cannot write to this display type %d characters! ", THIS_NODE_NAME, messageLength );
    return -9;
  }

  // We both limit check row and column and calculate cursor position per display spec
  uint8_t cursorPosition = CURSOR_LINE_1_START;
  if ((column < COLUMN_1_NUMBER) || (column > (NUM_CHARS_PER_LINE-1))) {
    ROS_ERROR("%s: Illegal column number of %d must be between %d and %d! ", THIS_NODE_NAME,
              column, COLUMN_1_NUMBER, (NUM_CHARS_PER_LINE-1) );
    return -8;
  }

  switch (row) {
      case LINE_1_ROW_NUMBER:
          cursorPosition = CURSOR_LINE_1_START + (column - 1);
          break;

      case LINE_2_ROW_NUMBER:
          cursorPosition = CURSOR_LINE_2_START + (column - 1);
          break;

      default:
          ROS_ERROR("%s: Illegal row number of %d must be  %d or %d! ", THIS_NODE_NAME,
               row, LINE_1_ROW_NUMBER, LINE_2_ROW_NUMBER);
          return -7;
          break;
  }
          
  // Now open serial port to get ready to update display
  int spFd = openSerialPort(g_serialPortName.c_str());
  if (spFd < 0) {
    ROS_ERROR("%s: Cannot open serial port for write to char display! ", THIS_NODE_NAME );
    return -1;
  }

  // If we are to clear full display do that first
  if (attributes & DISPLAY_ATTR_CLEAR_FIRST) {
      outBuf[0] = cursorPosition;
      bytesSent = write(spFd, outBuf, 1);    // Write all  bytes to the uart and thus the controller
      if (bytesSent != 1) {   // Write all  bytes to the uart and thus the controller
        ROS_ERROR("%s: Only wrote %d bytes out of %d for display cursor positioning!", 
            THIS_NODE_NAME, bytesSent, 1);
        return -2;
      }
      usleep(10000);     // Spec says wait 5ms after a clear so we double that
  }

  // Now we always set cursor first before write of characters
  outBuf[0] = cursorPosition;
  bytesSent = write(spFd, outBuf, 1);    // Write all  bytes to the uart and thus the controller
  if (bytesSent != 1) {   // Write all  bytes to the uart and thus the controller
    ROS_ERROR("%s: Only wrote %d bytes out of %d for display cursor positioning!", 
        THIS_NODE_NAME, bytesSent, 1);
    return -2;
  }
  usleep(10000);     // Wait a little bit even though spec does not require it

  // Now send the actual characters
  strncpy(&outBuf[0], text.c_str(), messageLength);
  int idx;
  for (idx=0 ; idx < messageLength ; idx++ ) {
      bytesSent = write(spFd, &outBuf[idx], 1);    // Write all  bytes to the uart and thus the controller
      if (bytesSent != 1) {   // Write all  bytes to the uart and thus the controller
          ROS_ERROR("%s: Failure to write text out for display update!", THIS_NODE_NAME );
          return -3;
      }
      usleep(1400);     // TODO: Fix issue if not waiting in driver for Rx Ready so we need wait
  }

  close (spFd);
  return 0;
}

// Driver to set the power on default message for the display
int displaySetStartupString(int line, std::string text, int semLock)
{
  // THIS DISPLAY DOES NOT HAVE BUILT IN STARTUP STRING
  return -1;
}


// Driver to set the LCD backlight brightness
int displaySetBrightness(int brightness, int semLock)
{
  // THIS DISPLAY CANNOT CONTROL LEVELS OF BRIGHTNESS
  return -1;
}


/**
 * Receive messages for display output
 */
void lcdDisplayApiCallback(const ros_bits::LcdModtronixMsg::ConstPtr& msg)
{
  ROS_DEBUG("%s heard display output msg: of actionType %d row %d column %d numChars %d attr 0x%x text %s comment %s]",
                THIS_NODE_NAME, msg->actionType, msg->row, msg->column, msg->numChars, msg->attributes, 
                msg->text.c_str(), msg->comment.c_str());

   // Now spool up this packet for background task
   switch (msg->actionType) {
     case MSG_DISPLAY_ALL:
     case MSG_DISPLAY_SUBSTRING:
       g_CmdQueue.push(DispCmd(msg->actionType, msg->row, msg->column, msg->numChars,
                       msg->attributes, msg->text.c_str(), msg->comment.c_str()));
       break;

     // For compatibility we eat these commands as do nothing commands
     case MSG_DISPLAY_STARTUP_STRING: 
     case MSG_DISPLAY_SET_BRIGHTNESS: 
       break;

     default:   // In fact we ignore ALL others but wanted to state others above for legacy
       break;
   } 
}

void charDisplayApiCallback(const ros_bits::CharDisplayMsg::ConstPtr& msg)
{
  ROS_DEBUG("%s heard display output msg: of actionType %d row %d column %d numChars %d attr 0x%x text %s comment %s]",
                THIS_NODE_NAME, msg->actionType, msg->row, msg->column, msg->numChars, msg->attributes, 
                msg->text.c_str(), msg->comment.c_str());

   // Now spool up this packet for background task
   switch (msg->actionType) {
     case MSG_DISPLAY_ALL:
     case MSG_DISPLAY_SUBSTRING:
       g_CmdQueue.push(DispCmd(msg->actionType, msg->row, msg->column, msg->numChars,
                       msg->attributes, msg->text.c_str(), msg->comment.c_str()));
       break;
     default:
       break;
   } 
}


//
// main:  This is the background processing loop    
//
// Pull display commands from the queue and update display
//
int main(int argc, char **argv)
{
  printf("ROS Node starting ros_bits:%s \n", THIS_NODE_NAME);

  // The ros::init() function initializes ROS and needs to see argc and argv
  ros::init(argc, argv, THIS_NODE_NAME);

  // Setup a NodeHandle for the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  // Define our main loop rate as a ROS rate and we will let ROS do waits between loops
  ros::Rate loop_rate(NODE_LOOPS_PER_SEC);

  // Set the display bootup string  and some other setup parameters each time we start
  std::string charDispSerialPort("bogusCommPort");   //  DEBUG!!!  CHAR_DISP_CONTROL_DEVICE);
  nh.getParam("/char_display/serial_port", charDispSerialPort);
  g_serialPortName = charDispSerialPort;
  ROS_INFO("%s: Using comm port %s \n", THIS_NODE_NAME, g_serialPortName.c_str());
  
  displayUpdate("    Display ", DISPLAY_ATTR_CLEAR_FIRST, 1, 1, 0);
  displayUpdate("    Started ", 0, 2, 1, 0);

  usleep(100000);

  ROS_INFO("%s: Parallax LCD Display subsystem ready! ", THIS_NODE_NAME);

  // Set to subscribe to the two display topics we support and we then get callbacks for each message
  ros::Subscriber subLcdDisp  = nh.subscribe(TOPIC_LCD_DISPLAY_OUTPUT, 1000, lcdDisplayApiCallback);
  ros::Subscriber subCharDisp = nh.subscribe(TOPIC_CHAR_DISPLAY_OUTPUT, 1000, charDisplayApiCallback);

  int loopCount = 0;
  while (ros::ok()) {

    if(!g_CmdQueue.empty())
    {
      DispCmd currentCmd;
      currentCmd = g_CmdQueue.front();
      g_CmdQueue.pop();

      ROS_INFO("%s: New Display CMD  '%s' [%d]  row %d column %d numChars %d Text %s",THIS_NODE_NAME,
         currentCmd.getCmdName().c_str(), currentCmd.actionType,
         currentCmd.row, currentCmd.column, currentCmd.numChars, currentCmd.text.c_str());

      // Now update display
      switch ( currentCmd.actionType ) {
        case MSG_DISPLAY_ALL:    // Clear display and output from 1st char of row 1
             currentCmd.attributes = currentCmd.attributes | DISPLAY_ATTR_CLEAR_FIRST;
             currentCmd.row = LINE_1_ROW_NUMBER;
             currentCmd.column = COLUMN_1_NUMBER;

             // This drop-through to next case is intentional!

        case MSG_DISPLAY_SUBSTRING: 
             displayUpdate(currentCmd.text, currentCmd.attributes,
                           currentCmd.row, currentCmd.column, currentCmd.numChars);
        break;
        default:  
        break;
      }
    }

    ros::spinOnce();                            // Give chance for callback reception

    loop_rate.sleep();
    ++loopCount;
    if (loopCount > 30000) { loopCount = 1; }
  }

  return 0;
}
