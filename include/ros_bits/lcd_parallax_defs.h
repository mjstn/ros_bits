#ifndef __CHAR_DISPLAY_DEFS_H
#define __CHAR_DISPLAY_DEFS_H

//
// Defines for a Parallax character based 2 line 16 char LCD display 
//
// This file defines specifics for the Parallax 2x16 LCD display
//

// The ROS topics we will use to send to the display node
// We will accept the LCD Modtronix message on lcd_display_output too so I can swap in one display or another
#define  TOPIC_CHAR_DISPLAY_OUTPUT    "char_display_output"
#define  TOPIC_LCD_DISPLAY_OUTPUT     "lcd_display_output"      // Depreciated display topic but we take it too

#define  NUM_LINES                      2
#define  NUM_CHARS_PER_LINE            16
#define  CHAR_DISPLAY_MAX_CHARS        (NUM_LINES*NUM_CHARS_PER_LINE)
#define  COLUMN_1_NUMBER                1   
#define  LINE_1_ROW_NUMBER              1   
#define  CURSOR_LINE_1_START           0x80
#define  LINE_2_ROW_NUMBER              2   
#define  CURSOR_LINE_2_START           0x94


// ---------------------------------------------------------------------------
// Here are non-hardware specific or message layer defines we use

// Used for the display of messages on the system hardware display
#define  MSG_DISPLAY_ALL               1        // The entire display (all lines) written with just this info
#define  MSG_DISPLAY_SUBSTRING         2        // Here we display on a single line N bytes of the message
#define  MSG_DISPLAY_STARTUP_STRING    3        // A dont-care but allow the command and do nothing
#define  MSG_DISPLAY_SET_BRIGHTNESS    4        // A dont-care but allow the command and do nothing

// Display message attributes
#define  DISPLAY_ATTR_CLEAR_FIRST         0x0001
#define  DISPLAY_ATTR_SET_CURSOR          0x0002
#define  DISPLAY_ATTR_BACKLIGHT_ON        0x0100
#define  DISPLAY_ATTR_BACKLIGHT_OFF       0x0200


// ---------------------------------------------------------------------------
// Serial display control code defines defined for the Parallax serial display

// These are their own command and need values after them
#define  PARLX_CMD_CLEAR_DISPLAY              0x0C    // Clear all chars and cursor to 1st char of line 1
#define  PARLX_CMD_CURSOR_LEFT                0x08    // move cursor back but do NOT erase char 
#define  PARLX_CMD_CURSOR_RIGHT               0x09    // move cursor right but do NOT erase char
#define  PARLX_CMD_BACKLIGHT_ON               0x11
#define  PARLX_CMD_BACKLIGHT_OFF              0x12
#define  PARLX_CMD_CURSOR_TO_HOME             0x80    // Add char position 0-32 for other places
#define  PARLX_CMD_SET_CURSOR_HOME            0x8b
#define  PARLX_CMD_SET_STARTUP_STRING         0x90


#endif  //  __CHAR_DISPLAY_DEFS_H
