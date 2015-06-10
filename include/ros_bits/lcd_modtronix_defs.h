#ifndef __LCD_MODTRONIX_DEFS_H
#define __LCD_MODTRONIX_DEFS_H

//
// Defines for Modtronix LCD display used over I2C
//
// This file defines specifics of the Modtronix LCD display interface
//

// The ROS topic we will use to send to the display node
#define  TOPIC_LCD_DISPLAY_OUTPUT     "lcd_display_output"

#define  I2C_LCD_DISPLAY_MAX_CHARS        32


// ---------------------------------------------------------------------------
// Here are non-hardware specific or message layer defines we use

// Used for the display of messages on the system hardware display
#define  MSG_DISPLAY_ALL               1        // The entire display (all lines) written with just this info
#define  MSG_DISPLAY_SUBSTRING         2        // Here we display on a single line N bytes of the message
#define  MSG_DISPLAY_STARTUP_STRING    3        // Set the power on bootup string
#define  MSG_DISPLAY_SET_BRIGHTNESS    4        // Set the display brightness

// Display message attributes
#define  DISPLAY_ATTR_CLEAR_FIRST         0x0001
#define  DISPLAY_ATTR_SET_CURSOR          0x0002
#define  DISPLAY_ATTR_BACKLIGHT_ON        0x0100
#define  DISPLAY_ATTR_BACKLIGHT_OFF       0x0200


// ---------------------------------------------------------------------------
// Serial display I2C defines defined for the Modtronix LCD3S serial display
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


#endif   // __LCD_MODTRONIX_DEFS_H
