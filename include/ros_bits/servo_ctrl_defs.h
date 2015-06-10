#ifndef __SERVO_CTRL_HW_DEFS_H
#define __SERVO_CTRL_HW_DEFS_H

//
// Defines that tie in specific of hardware and the serial port usage
//
// Module supplied with no warenty from www.mark-toys.com
//

// If you have custom hardware support for a reset line to the servo controller 
// then define this below AND code for your reset line in the servo_ctrl_server source code
#undef  SERVO_RESET_LINE

// These defines are  owned by servo controller node but here for all-in-one hardware limits
// Support up to 4 controllers. Controllers ignore channels they don't support via jumpers
// Each controller has 8 servo outputs but the channel +8 is for wider range of control
// This module is fairly blind to the odd rules but that logic 'could' be in this server
#define SERVO_POLOLU_PROTOCOL       // if not defined we use 3-byte  SERVO_MINI_SSC_II_MODE

#define SERVO_MAX_CHANNEL    6      // Maximum servo channels from 1 that we support
#define SERVO_MIN_POSITION   0
#define SERVO_MAX_POSITION   240
#define SERVO_SLEW           4      // Pololu slew rate. 1 to 127

#endif  // __SERVO_CTRL_HW_DEFS_H
