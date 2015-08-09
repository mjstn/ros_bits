#ifndef __MOTOR_CONTROL_DEFS_H
#define __MOTOR_CONTROL_DEFS_H

//
// Defines for motor controller node to drive the Mark-Toys motor controller board over I2C
//

// The ROS topic we will use to send to the display node
#define  TOPIC_WHEEL_CONTROL     "wheel_control"   // Motor driver input commands


// The command types to the motor control node
#define  MSG_MOTOR_CONTROL_STOP        0
#define  MSG_MOTOR_CONTROL_RIGHT       1
#define  MSG_MOTOR_CONTROL_LEFT        2
#define  MSG_MOTOR_CONTROL_BOTH        3

#define  MSG_MOTOR_RIGHT_NUMBER        1
#define  MSG_MOTOR_LEFT_NUMBER         2
#define  MSG_MOTOR_BOTH_NUMBER         3

// For a motor we can tell it to go forward, backward or other modes like breaking
#define  MSG_MOTOR_DIRECTION_FORWARD   1
#define  MSG_MOTOR_DIRECTION_REVERSE   -1
#define  MSG_MOTOR_BREAK_GND           2
#define  MSG_MOTOR_BREAK_VCC           3


// Defines below are for the Mark-Toys motor controller that drives dual VNH2SP30 motor controller
#define  MT_MOT_CTRL_CMD_SET_PWM      0x50      // Set channel as a 0-100 pct PWM value
#define  MT_MOT_CTRL_CMD_SET_MOTOR    0x51      // Set channel as a 0-100 pct PWM value and set ctrl 2 bits
#define  MT_MOT_CTRL_CMD_SET_HR_SERVO 0x52      // Set a servo 0-1000 for 0-2.5ms 
#define  MT_MOT_CTRL_CMD_SET_LR_SERVO 0x53      // Set a servo 0-255 for 0-2.5ms (Low Resolution)

#define  MT_MOT_CTRL_RIGHT_MOTOR       1        // The channel for the right motor
#define  MT_MOT_CTRL_LEFT_MOTOR        2        // The channel for the left  motor

// Control bits for the motors
// Think of this as LSB is INb and MSB is INa for a 2-bit value
#define  MT_MOT_CTRL_BREAK_TO_GND      0
#define  MT_MOT_CTRL_DIR_CTR_CLOCKWISE 1
#define  MT_MOT_CTRL_DIR_CLOCKWISE     2
#define  MT_MOT_CTRL_BREAK_TO_VCC      3

#endif  // __MOTOR_CONTROL_DEFS_H

