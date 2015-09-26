#ifndef __SERVO_CTRL_HW_DEFS_H
#define __SERVO_CTRL_HW_DEFS_H

// ROS Server To run Servos using Pololu or Mark-Toys Servo Controller
//
// As of late 2015 this node can also support the Mark-Toys motor/servo controller.
// The Mark-toys.com controller is accessed over I2C and has two section (two separate I2C controllers)
// Each controller can be setup for being a 3-channel servo controller (or a motor controller)
// To use this you must therefore have the side being used to have correct firmware.
//
// Defines that tie in specific of hardware and the serial port usage
//
// Module supplied with no warenty from www.mark-toys.com
//

// If you have custom hardware support for a reset line to the servo controller 
// then define this below AND change code for your reset line in servo_ctrl_server.cpp
#define  SERVO_RESET_LINE


// IMPORTANT HARD CONFIG FOR THIS SERVO CONTROL MODULE
//
// You MUST compile this node for use with the Mark-Toys servo controller or
// one of two modes for the Pololu serial input servo controller (pre USB controller)
//
#define POLOLU_USING_POLOLU_PROTOCOL        1
#define POLOLU_USING_INDUSTRY_PROTOCOL      2
#define MARK_TOYS_CONTROLLER                3
        
#define SERVO_CTRL_HW_TYPE   MARK_TOYS_CONTROLLER

// These defines are  owned by servo controller node but here for all-in-one hardware limits
// Support up to 4 controllers. Controllers ignore channels they don't support via jumpers
// Each controller has 8 servo outputs but the channel +8 is for wider range of control
// This module is fairly blind to the odd rules but that logic 'could' be in this server
#define SERVO_POLOLU_PROTOCOL       // if not defined we use 3-byte  SERVO_MINI_SSC_II_MODE



#define POLOLU_SERVO_MIN_CHANNEL    1      // Minimum servo channel to be supported
#define POLOLU_SERVO_MAX_CHANNEL    6      // Maximum servo channels from 1 that we support
#define POLOLU_SERVO_MIN_POSITION   0
#define POLOLU_SERVO_MAX_POSITION   240
#define SERVO_SLEW           20     // Pololu slew rate. 1 (slow) to 127 (fast)  0=instant

#define MARKTOYS_SERVO_MIN_CHANNEL    1      // Minimum servo channel to be supported
#define MARKTOYS_SERVO_MAX_CHANNEL    6      // Maximum servo channels from 1 that we support
#define MARKTOYS_SERVO_MIN_POSITION   0
#define MARKTOYS_SERVO_MAX_POSITION   240

// Command info for control of Mark-Toys servo controller
#define  MTOY_CMD_SET_PERIOD   0xA1      // Set new pwm divisor that follows in next 2 bytes
#define  MTOY_CMD_SET_PWM      0x50      // Set channel in next byte as a 0-100 pct PWM value in next byte
#define  MTOY_CMD_SET_MOTOR    0x51      // Set channel in next byte as a 0-100 pct PWM value and ctrl 2 bits next byte
#define  MTOY_CMD_SET_HR_SERVO 0x52      // Set channel in next byte as a servo 0-1000 for 0-2.5ms in next 2 bytes
#define  MTOY_CMD_SET_LR_SERVO 0x53      // Set channel in next byte as a servo 255 for 0-2.5ms in next byte

#endif  // __SERVO_CTRL_HW_DEFS_H
