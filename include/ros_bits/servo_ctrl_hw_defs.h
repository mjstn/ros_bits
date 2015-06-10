//
// Defines that tie in specific of hardware and the serial port usage
//
// Module supplied with no warenty from www.mark-toys.com
//

// Serial port for PWM board control 
// If you use usb it may end up as  /dev/ttyUSB0 and so on.
//
// TODO: The servo tty port would be best to come from a ROS parameter
//
#ifdef   HW_BB_BLACK
#define  SERVO_PWM_CONTROL_DEVICE          "/dev/ttyO2"    // custom device tree required in am335x-boneblack.dtb
#else
#define  SERVO_PWM_CONTROL_DEVICE          "/dev/ttyAMA0"  // Raspberry Pi default port
#endif

// These defines are  owned by servo controller node but here for all-in-one hardware limits
// Support up to 4 controllers. Controllers ignore channels they don't support via jumpers
// Each controller has 8 servo outputs but the channel +8 is for wider range of control
// This module is fairly blind to the odd rules but that logic 'could' be in this server
#define SERVO_POLOLU_PROTOCOL       // if not defined we use 3-byte  SERVO_MINI_SSC_II_MODE

#define SERVO_MAX_CHANNEL    6      // Maximum servo channels from 1 that we support
#define SERVO_MIN_POSITION   0
#define SERVO_MAX_POSITION   240
#define SERVO_SLEW           4      // Pololu slew rate. 1 to 127

