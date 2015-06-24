/*
 * Servo Controller to access the Pololu Micro Serial Servo Controller
 *
 * We will 'own' a single serial port and this module serializes access.
 *
 * This module only knows there are some number of servo channels and does
 * not nor should it know what those channels are being used for at any given time
 *
 * Author:    Mark Johnston
 * Creation:  20150605    Initial creation of this module from a prior Mark-Toys.com project
 */
#define  THIS_SERVER_NAME    "servo_ctrl_service"   // The Name for this ROS service

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include "ros/ros.h"

#include "ros_bits/serial_common_defs.h"        // Defines for system serial port to use for control
#include "ros_bits/servo_ctrl_defs.h"           // Defines used for servo control module

#include "ros_bits/ServoCtrlSrv.h"              // ROS service defines

// We use the same call open helper call
int openSerialPort(void) {
  int   spFd;                                   // File descriptor for serial port
  const char  *spPortName = SERVO_PWM_CONTROL_DEVICE; // hardware device for the serial port
  spFd = open(spPortName, O_RDWR | O_NOCTTY);   // open port for read/write but not as controlling terminal
  if (spFd < 0) {
    ROS_ERROR("%s: Error in open of serial port '%s' for servo controller! ", THIS_SERVER_NAME, spPortName);
    // we just return the bad file descriptor also as an error
  }
  return spFd;
}

// Initialize the servo board which for this board
// means send a character to set it's baud rate
int initServoHardware() {
  uint8_t outBuf[8];
  int retCode = 0;
  int fd;

  // Do a test to see if we can open the serial port
  ROS_INFO("%s: Check that we can open serial port.", THIS_SERVER_NAME);
  fd = openSerialPort();
  if (fd < 0)  {
    ROS_ERROR("%s: Error in opening of serial port for servo controller hardware ", THIS_SERVER_NAME);
    return -1;
  }

  ROS_INFO("%s: Setup servo baud rate with auto detect ", THIS_SERVER_NAME);

  #ifndef SERVO_POLOLU_PROTOCOL   // {
  // Send a simple Mini SSC II Mode command to set servo number 6 just as a baud rate setup 
  // Frankly I wish I could set it with dip switch but it is sadly 'too smart for it's own good'
  outBuf[0] = 0xff;
  outBuf[0] = 0x07;
  outBuf[0] = 0x80;
  if ((write(fd, outBuf, 3)) != 3) {             // Write all  bytes to the uart and thus the controller
    ROS_ERROR("%s: Error in initial writing to serial port for initServoHardware() ", THIS_SERVER_NAME);
    retCode = -2;
  }
  #endif

  #ifdef SERVO_RESET_LINE        // {
  // Code for custom servo board reset would go here if support desired
  // }
  #endif // SERVO_RESET_LINE

  #ifdef SERVO_POLOLU_PROTOCOL   // {

  ROS_INFO("%s: Setup servo  PoloLu mode ", THIS_SERVER_NAME);
  // Need to setup controller if in more complex POLOLU protocol mode
  outBuf[0] = 0x80;	    // The Pololu Start Byte
  outBuf[1] = 0x01;         // Device ID which is 1 for micro 8 channel controller;
  outBuf[2] = 0;            // Command 0:  Set parameters 
  outBuf[4] = 0x50;         // Servo On, Direction Fwd, 5 bit servo range 

  for (int c=SERVO_MIN_CHANNEL ; (retCode == 0) && (c <= SERVO_MAX_CHANNEL) ; c++) {
    outBuf[3] = c;          // Servo channel being setup
    if ((write(fd, outBuf, 5)) != 5) {             // Write all  bytes to the uart and thus the controller
      retCode = -5;
      break;
    }
  }
  
  // Now set slew to not be so harsh for slower servo movements
  outBuf[0] = 0x80;	    // The Pololu Start Byte
  outBuf[1] = 0x01;         // Device ID which is 1 for micro 8 channel controller;
  outBuf[2] = 1;            // Set speed of slew 
  outBuf[4] = SERVO_SLEW;   // 1 is 50 usec per sec, 127 is max slew speed

  for (int c=SERVO_MIN_CHANNEL ; (retCode == 0) && (c <= SERVO_MAX_CHANNEL) ; c++) {
    outBuf[3] = c;          // Servo channel being setup
    if ((write(fd, outBuf, 5)) != 5) {             // Write all  bytes to the uart and thus the controller
      retCode = -6;
      break;
    }
  }

  close(fd);

  #endif   // }

  if (retCode == 0) {
    ROS_INFO("%s: Servo board has been setup. ", THIS_SERVER_NAME);
  } else {
    ROS_ERROR("%s: Error %d in initServoHardware() ", THIS_SERVER_NAME, retCode);
  }

  return retCode;
}

// Low level servo controller call
int setServoHardware(int fd, int channel, int position) {
  uint8_t outBuf[6];
  int bytesSent = 0;
  int retCode = 0;

  if ((channel < SERVO_MIN_CHANNEL) || (channel > SERVO_MAX_CHANNEL)) {
    ROS_ERROR("%s: Illegal servo channel of %d! ", THIS_SERVER_NAME, channel);
    return -1;
  }

  // We could chose to customize each channel for real-world units
  // and this is the place to do that and check at the same time
  if ((position < SERVO_MIN_POSITION) || (position > SERVO_MAX_POSITION)) {
    ROS_ERROR("%s: Illegal servo position of %d! ", THIS_SERVER_NAME, position);
    return -2;
  }

  int byteCount = 0;
#ifdef SERVO_POLOLU_PROTOCOL
  outBuf[0] = 0x80;	// Use the Pololu protocol(s)
  outBuf[1] = 0x01;     // Device ID which is 1 for micro 8 channel controller;
  outBuf[2] = 3;        // Set position in 8-bit Pololu mode 
  outBuf[3] = channel;
  outBuf[4] = (position >> 7) & 1;    // LSB contains the most significant bit of position
  outBuf[5] = (0x7F & position);  // 7 least significant bits
  byteCount = 6;
#else
  outBuf[0] = 0xff;	// The Mini SSC II mode 
  outBuf[1] = channel;
  outBuf[3] = position;
  byteCount = 3;
#endif

  bytesSent = write(fd, outBuf, byteCount);    // Write all  bytes to the uart and thus the controller
  if (bytesSent != byteCount) {   // Write all  bytes to the uart and thus the controller
    ROS_ERROR("%s: Only wrote %d bytes out of %d for servo channel of %d! ", THIS_SERVER_NAME,
        bytesSent, byteCount, channel);
    retCode = -9;
  }

  return retCode;
}

// Initialize servo controller for some other node in the system
// Note that this routine alone cannot do a hard reset programatically as that
// ability lies with another higher level node
bool initServoController(ros_bits::ServoCtrlSrv::Request  &req, 
                         ros_bits::ServoCtrlSrv::Response &res)
{
  // the parameters for the request are not used so we don't read them

  // Initialize servo controller
  if (initServoHardware() != 0)  {
    ROS_ERROR("%s: Error in servo controller hardware initialization from initServoController ", THIS_SERVER_NAME);
    return false;
  }

  return true;
}

// setOneServo only pays attention to the servo A parameters
bool setOneServo(ros_bits::ServoCtrlSrv::Request  &req, 
                 ros_bits::ServoCtrlSrv::Response &res)
{
  int servoChannel  = (int)req.servoAChannel;
  int servoPosition = (int)req.servoAPosition;

  ROS_INFO("%s: setOneServo request: set servo %d to position %d ", THIS_SERVER_NAME, servoChannel, servoPosition);

  int spFd = openSerialPort();
  if (spFd < 0) {
    ROS_ERROR("%s: Cannot open serial port for write to servo channel of %d! ", THIS_SERVER_NAME, servoChannel);
    return false;
  }

  // Set the PWM hardware for this one servo 
  int retCode = setServoHardware(spFd, servoChannel, servoPosition);
  ROS_INFO("%s: setOneServo request: set servo %d to position %d had retCode %d", THIS_SERVER_NAME,
          servoChannel, servoPosition, retCode);

  close(spFd);

  return (retCode == 0) ? true : false;
}

// setTwoServos sets two servos for cases where they need to be set close to each other
bool setTwoServos(ros_bits::ServoCtrlSrv::Request  &req,
                  ros_bits::ServoCtrlSrv::Response &res)
{
  int servoAChannel  = (int)req.servoAChannel;
  int servoAPosition = (int)req.servoAPosition;
  int servoBChannel  = (int)req.servoBChannel;
  int servoBPosition = (int)req.servoBPosition;

  ROS_INFO("%s: setTwoServos: request: set servo %d to pos %d and servo %d to pos %d ", THIS_SERVER_NAME,
           servoAChannel, servoAPosition, servoBChannel, servoBPosition);

  int spFd = openSerialPort();
  if (spFd < 0) {
    return false;
  }

  // Set the PWM hardware for the first servo then the other
  int retCodeA = setServoHardware(spFd, servoAChannel, servoAPosition);
  int retCodeB = setServoHardware(spFd, servoBChannel, servoBPosition);

  close(spFd);

  return (retCodeA == retCodeB == 0) ? true : false;
}


//
// This is the main node loop for this ROS service
//
int main(int argc, char **argv)
{
  // The ros::init() function initializes ROS and needs to see argc and argv
  ros::init(argc, argv, "servo_controller_server");

  // Setup a NodeHandle for the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  if (initServoHardware() != 0)  {
    ROS_ERROR("%s: Error in servo controller hardware initialization ", THIS_SERVER_NAME);
    return -2;
  }

  ros::ServiceServer service0 = nh.advertiseService("init_servo_controller", initServoController);
  ros::ServiceServer service1 = nh.advertiseService("set_one_servo", setOneServo);
  ros::ServiceServer service2 = nh.advertiseService("set_two_servos", setTwoServos);

  ROS_INFO("%s: Ready to accept server requests.", THIS_SERVER_NAME);

  // ros::spin() will enter a loop allowing callbacks to happen. Exits on Ctrl-C
  ros::spin();

  return 0;
}
