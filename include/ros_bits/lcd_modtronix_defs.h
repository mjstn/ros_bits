//
// Defines for Modtronix LCD display used over I2C
//
// This file defines specifics of the Modtronix LCD display interface
//

// The ROS topic we will use to send to the display node
#define  TOPIC_LCD_DISPLAY_OUTPUT     "lcd_display_output"

#define  I2C_LCD_DISPLAY_MAX_CHARS        32

// Serial display I2C defines for Modtronix LCD3S serial display
// Most of these follow the 'command' value of 0x80.
// There are MANY of these not supported yet and some below not implemented yet
#define  LCD3S_CLEAR_ALL                  0x0c

// These are their own command and need values after them
#define  LCD3S_CMD_SET_TEXT_AT_CURSOR         0x80
#define  LCD3S_CMD_BACKLIGHT_LEVEL            0x81
#define  LCD3S_CMD_CLEAR_DISPLAY              0x8c
#define  LCD3S_CMD_SET_CURSOR_POSITION        0x8a
#define  LCD3S_CMD_SET_CURSOR_HOME            0x8b
#define  LCD3S_CMD_SET_STARTUP_STRING         0x90


