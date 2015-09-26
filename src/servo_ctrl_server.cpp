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
#define  THIS_NODE_NAME    "servo_ctrl_service"   // The Name for this ROS service

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>

#include "ros/ros.h"

#include "ros_bits/serial_common_defs.h"        // Defines for system serial port to use for control
#include "ros_bits/servo_ctrl_defs.h"           // Defines used for servo control module

#include "ros_bits/ServoCtrlSrv.h"              // ROS service defines

#ifdef SERVO_RESET_LINE        // {
#include "ros_bits/GPIOClass.h"                 // Pull in GPIOClass which you must have to use SERVO_RESET_LINE
#define  SERVO_RESET_GPIO_PIN   "27"            // The pin we will use for servo reset
#include <ros_bits/GPIOClass.cpp>               // Do a straight inline of the GPIO class code
#endif // SERVO_RESET_LINE


// Define a semaphore lock for I2C that will be initialized in main and called from callback
// We pull in semaphore code inline rather than a lib as it is very minimal code
#include <ros_bits/i2c_common_defs.h>             // I2C shared hardware bus defines
#include <ros_bits/ipc_sems.cpp>         // We basically inline the sem calls
int g_i2c_semaphore_id;


// We use the same call open helper call
// Returns file descriptor if successful
int openSerialPort(void) {
  int   spFd;                                   // File descriptor for serial port
  const char  *spPortName = SERVO_PWM_CONTROL_DEVICE; // hardware device for the serial port
  spFd = open(spPortName, O_RDWR | O_NOCTTY);   // open port for read/write but not as controlling terminal
  if (spFd < 0) {
    ROS_ERROR("%s: Error in open of serial port '%s' for servo controller! ", THIS_NODE_NAME, spPortName);
    // we just return the bad file descriptor also as an error
  }
  return spFd;
}

// Initialize the Pololu specific servo board which for this board
// means send a character to set it's baud rate
//
int initPololuServoHardware() {
  int     retCode = 0;
  uint8_t outBuf[8];
  int     spFd = 0;

  // Do a test to see if we can open the serial port
  ROS_INFO("%s: Check that we can open serial port.", THIS_NODE_NAME);
  spFd = openSerialPort();
  if (spFd < 0)  {
    ROS_ERROR("%s: Error in opening of serial port for servo controller hardware ", THIS_NODE_NAME);
    return -1;
  }

  if (SERVO_CTRL_HW_TYPE == POLOLU_USING_INDUSTRY_PROTOCOL) {
    ROS_INFO("%s: Setup servo baud rate with auto detect ", THIS_NODE_NAME);
      // Send a simple Mini SSC II Mode command to set servo number 6 just as a baud rate setup 
      // Frankly I wish I could set it with dip switch but it is sadly 'too smart for it's own good'
      outBuf[0] = 0xff;
      outBuf[0] = 0x07;
      outBuf[0] = 0x80;
      if ((write(spFd, outBuf, 3)) != 3) {             // Write all  bytes to the uart and thus the controller
        ROS_ERROR("%s: Error in initial writing to Pololu servo controller ", THIS_NODE_NAME);
        retCode = -2;
      }
  }

  // --------------------------------
  #ifdef SERVO_RESET_LINE        // {

  // Code for custom servo board reset would go here if support desired
  ROS_INFO("%s: Reset servo board ", THIS_NODE_NAME);

  GPIOClass* servoResetPort = new GPIOClass(SERVO_RESET_GPIO_PIN);
  servoResetPort->export_gpio();
  servoResetPort->setdir_gpio("out");

  // Now bring the line low then high again
  servoResetPort->setval_gpio("0");
  ros::Duration(0.1).sleep();
  servoResetPort->setval_gpio("1");
  ros::Duration(0.2).sleep();

  #endif // } SERVO_RESET_LINE
  // --------------------------------

  if (SERVO_CTRL_HW_TYPE == POLOLU_USING_POLOLU_PROTOCOL) {
    ROS_INFO("%s: Setup servo  PoloLu mode ", THIS_NODE_NAME);
    // Need to setup controller if in more complex POLOLU protocol mode
    outBuf[0] = 0x80;	    // The Pololu Start Byte
    outBuf[1] = 0x01;         // Device ID which is 1 for micro 8 channel controller;
    outBuf[2] = 0;            // Command 0:  Set parameters 
    outBuf[4] = 0x50;         // Servo On, Direction Fwd, 5 bit servo range 

    for (int c=POLOLU_SERVO_MIN_CHANNEL ; (retCode == 0) && (c <= POLOLU_SERVO_MAX_CHANNEL) ; c++) {
      outBuf[3] = c;          // Servo channel being setup
      if ((write(spFd, outBuf, 5)) != 5) {             // Write all  bytes to the uart and thus the controller
        retCode = -5;
        break;
      }
    }
    // Now set slew to not be so harsh for slower servo movements
    outBuf[0] = 0x80;	    // The Pololu Start Byte
    outBuf[1] = 0x01;         // Device ID which is 1 for micro 8 channel controller;
    outBuf[2] = 1;            // Set speed of slew 
    outBuf[4] = SERVO_SLEW;   // 1 is 50 usec per sec, 127 is max slew speed

    for (int c=POLOLU_SERVO_MIN_CHANNEL ; (retCode == 0) && (c <= POLOLU_SERVO_MAX_CHANNEL) ; c++) {
      outBuf[3] = c;          // Servo channel being setup
      if ((write(spFd, outBuf, 5)) != 5) {             // Write all  bytes to the uart and thus the controller
        retCode = -6;
        break;
      }
    }

  }

  close(spFd);

  if (retCode == 0) {
    ROS_INFO("%s: Servo board has been setup. ", THIS_NODE_NAME);
  } else {
    ROS_ERROR("%s: Error %d in initServoHardware() ", THIS_NODE_NAME, retCode);
  }

  return retCode;
}


// Pololu servo controller specific Low level driver
//
// Returns 0 for all was ok or code <= -10 for controller specific fault
//
int setPololuServoHardware(int channel, int position) {
  int retCode = 0;
  int fd;

  if ((channel < POLOLU_SERVO_MIN_CHANNEL) || (channel > POLOLU_SERVO_MAX_CHANNEL)) {
    ROS_ERROR("%s: Illegal servo channel of %d! ", THIS_NODE_NAME, channel);
    return -10;
  }

  // We could chose to customize each channel for real-world units
  // and this is the place to do that and check at the same time
  if ((position < POLOLU_SERVO_MIN_POSITION) || (position > POLOLU_SERVO_MAX_POSITION)) {
    ROS_ERROR("%s: Illegal servo position of %d! ", THIS_NODE_NAME, position);
    return -11;
  }

  int spFd = openSerialPort();
  if (spFd < 0) {
    ROS_ERROR("%s: Cannot open serial port for write to servo channel of %d! ", THIS_NODE_NAME, channel);
    return false;
  }

  int byteCount = 0;
  int bytesSent = 0;
  uint8_t outBuf[8];
  
  switch (SERVO_CTRL_HW_TYPE) {
    case POLOLU_USING_POLOLU_PROTOCOL:
      outBuf[0] = 0x80;	// Use the Pololu protocol(s)
      outBuf[1] = 0x01;     // Device ID which is 1 for micro 8 channel controller;
      outBuf[2] = 3;        // Set position in 8-bit Pololu mode 
      outBuf[3] = channel;
      outBuf[4] = (position >> 7) & 1;    // LSB contains the most significant bit of position
      outBuf[5] = (0x7F & position);  // 7 least significant bits
      byteCount = 6;
      break;

    case POLOLU_USING_INDUSTRY_PROTOCOL:
      outBuf[0] = 0xff;	// The Mini SSC II mode 
      outBuf[1] = channel;
      outBuf[3] = position;
      byteCount = 3;
      break;

    default:
      retCode = -13;
      break;
  }

  // Now send the bytes to the controller
  if (retCode == 0) {
    bytesSent = write(fd, outBuf, byteCount);    // Write all  bytes to the uart and thus the controller
    if (bytesSent != byteCount) {   // Write all  bytes to the uart and thus the controller
      ROS_ERROR("%s: Only wrote %d bytes out of %d for servo channel of %d! ", THIS_NODE_NAME,
        bytesSent, byteCount, channel);
      retCode = -19;
    }
  }

  close(spFd);

  return retCode;
}

// MarkToys servo controller specific Low level driver
//
// Returns 0 for all was ok or code <= -10 for controller specific fault
//
int setMarkToysServoHardware(int channel, int position) {
  int retCode = 0;

  if ((channel < MARKTOYS_SERVO_MIN_CHANNEL) || (channel > MARKTOYS_SERVO_MAX_CHANNEL)) {
    ROS_ERROR("%s: Illegal servo channel of %d! ", THIS_NODE_NAME, channel);
    return -10;
  }

  // We could chose to customize each channel for real-world units
  // and this is the place to do that and check at the same time
  if ((position < MARKTOYS_SERVO_MIN_POSITION) || (position > MARKTOYS_SERVO_MAX_POSITION)) {
    ROS_ERROR("%s: Illegal servo position of %d! ", THIS_NODE_NAME, position);
    return -11;
  }

  int i2cAddress;
  int semLock = g_i2c_semaphore_id;

  // To send data over I2C we must aquire a OS lock
  if (ipc_sem_lock(semLock) < 0) {
      ROS_ERROR("%s: servo driver sem lock error!\n", THIS_NODE_NAME);
      return -12;
  }

  int fd;                                       // File descrition
  unsigned char buf[10];                        // Buffer for data being read/ written on the i2c bus
  const char *i2cDevName = I2C_DEV_FOR_SERVO_CONTROL;   // Name of the port we will be using

  if ((fd = open(i2cDevName, O_RDWR)) < 0) {    // Open port for reading and writing
    if (semLock >= 0) {ipc_sem_unlock(semLock);}
    return -13;
  }

  i2cAddress = I2C_SERVO_CONTROL_REG_ADDR;              // Address of motor controller on I2C
  if (ioctl(fd, I2C_SLAVE, i2cAddress) < 0) {   // Set the port options and addr of the dev
    printf("Unable to get bus access to talk to slave servo controller over I2C\n");
    if (semLock >= 0) {ipc_sem_unlock(semLock);}  close(fd);
    close (fd);         // Close the I2C handle
    return -14;
  }

  buf[0] = MTOY_CMD_SET_LR_SERVO;   // Command byte to the controller
  buf[1] = channel & 0xff;;
  buf[2] = position & 0xff;;
  ROS_ERROR("%s: motorDriver Right Wheel Write Bytes: Addr 0x%x channel %d PwmPct %d Control %d ",
                THIS_NODE_NAME, i2cAddress, buf[1], buf[2], buf[3]);
  if ((write(fd, buf, 3)) != 3) {                       // Write commands to the i2c port
    printf("Error writing to i2c slave motor controller\n");
    retCode = -15;
  }

  if (semLock >= 0) {ipc_sem_unlock(semLock);}  
  close(fd);

  return retCode;
}


// Set the servo channel to a specific position
// Servo set hardware call that directs to servo specific driver
//
int setServoHardware(int channel, int position) {
  int retCode = 0;

  switch (SERVO_CTRL_HW_TYPE) {
    case POLOLU_USING_POLOLU_PROTOCOL:
    case POLOLU_USING_INDUSTRY_PROTOCOL:
      retCode = setPololuServoHardware(channel, position);
      break;
    case MARK_TOYS_CONTROLLER:
      retCode = setMarkToysServoHardware(channel, position);
      break;
    default:
      ROS_ERROR("%s: Illegal servo controller type of %d ", THIS_NODE_NAME, SERVO_CTRL_HW_TYPE);
      retCode = -9;
      break;
  }

  return retCode;
}


// Initialize servo hardware depending on the controller type
//
bool initServoHardware(void)
{
  switch (SERVO_CTRL_HW_TYPE) {
    case POLOLU_USING_POLOLU_PROTOCOL:
    case POLOLU_USING_INDUSTRY_PROTOCOL:
      if (initPololuServoHardware() != 0)  {
        ROS_ERROR("%s: Error in Pololu servo controller hardware init", THIS_NODE_NAME);
        return false;
      }
      break;
    case MARK_TOYS_CONTROLLER:
      // No specific init required for this controller as of yet
      break;
    default:
      ROS_ERROR("%s: Illegal servo controller type of %d ", THIS_NODE_NAME, SERVO_CTRL_HW_TYPE);
      return false;
      break;
  }

  return true;
}

// Initialize servo controller for some other node in the system
// Note that this routine alone cannot do a hard reset programatically as that
// ability lies with another higher level node
bool initServoController(ros_bits::ServoCtrlSrv::Request  &req, 
                         ros_bits::ServoCtrlSrv::Response &res)
{
  // Initialize servo controller
  return initServoHardware();
}


// setOneServo only pays attention to the servo A parameters
//
// Return less than 0 for any fault
//
bool setOneServo(ros_bits::ServoCtrlSrv::Request  &req, 
                 ros_bits::ServoCtrlSrv::Response &res)
{
  int servoChannel  = (int)req.servoAChannel;
  int servoPosition = (int)req.servoAPosition;

  ROS_INFO("%s: setOneServo request: set servo %d to position %d ", THIS_NODE_NAME, servoChannel, servoPosition);

  // Set the PWM hardware for this one servo 
  int retCode = setServoHardware(servoChannel, servoPosition);
  ROS_INFO("%s: setOneServo request: set servo %d to position %d had retCode %d", THIS_NODE_NAME,
          servoChannel, servoPosition, retCode);

  return (retCode == 0) ? true : false;
}

// setTwoServos sets two servos for cases where they need to be set close to each other
//
// Return less than 0 for any fault
//
bool setTwoServos(ros_bits::ServoCtrlSrv::Request  &req,
                  ros_bits::ServoCtrlSrv::Response &res)
{
  int servoAChannel  = (int)req.servoAChannel;
  int servoAPosition = (int)req.servoAPosition;
  int servoBChannel  = (int)req.servoBChannel;
  int servoBPosition = (int)req.servoBPosition;

  ROS_INFO("%s: setTwoServos: request: set servo %d to pos %d and servo %d to pos %d ", THIS_NODE_NAME,
           servoAChannel, servoAPosition, servoBChannel, servoBPosition);

  // Set the PWM hardware for the first servo then the other
  int retCodeA = setServoHardware(servoAChannel, servoAPosition);
  int retCodeB = setServoHardware(servoBChannel, servoBPosition);

  return (retCodeA == retCodeB == 0) ? true : false;
}


//
// main: This is the main node loop for this ROS service
//
int main(int argc, char **argv)
{
  // The ros::init() function initializes ROS and needs to see argc and argv
  ros::init(argc, argv, "servo_controller_server");

  // Setup a NodeHandle for the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  // Setup process global semaphore id for I2C lock and ioctl structs for using the sem
  // In this system our main node initializes this semaphore
  int semMaxWaitSec = 300;
  g_i2c_semaphore_id = -9;
  if (SEM_PROTECT_I2C == 0) {
    g_i2c_semaphore_id = IPC_SEM_DUMMY_SEMID;
    ROS_INFO("%s: I2C sem DUMMY lock used so user code can run \n", THIS_NODE_NAME);
  } else {
    ROS_INFO("%s: Wait for I2C sem lock for up to %d seconds or die.  \n", THIS_NODE_NAME,  semMaxWaitSec);
    g_i2c_semaphore_id = ipc_sem_get_by_key_with_wait(I2C_WHEEL_CONTROL_SEM_KEY, semMaxWaitSec);
    if (g_i2c_semaphore_id < 0) {
      ROS_INFO("%s: Cannot acquire lock for I2C! Skip Message! ", THIS_NODE_NAME);
      return -1;
    }
  }
  ROS_INFO("%s: I2C sem lock obtained as value %d \n", THIS_NODE_NAME,  g_i2c_semaphore_id);

  if (!initServoHardware())  {
    ROS_ERROR("%s: Error in servo controller hardware initialization ", THIS_NODE_NAME);
    return -2;
  }

  ros::ServiceServer service0 = nh.advertiseService("init_servo_controller", initServoController);
  ros::ServiceServer service1 = nh.advertiseService("set_one_servo", setOneServo);
  ros::ServiceServer service2 = nh.advertiseService("set_two_servos", setTwoServos);

  ROS_INFO("%s: Ready to accept server requests.", THIS_NODE_NAME);

  // ros::spin() will enter a loop allowing callbacks to happen. Exits on Ctrl-C
  ros::spin();

  return 0;
}
