#ifndef __PROX_MULTI_SENSOR_DEFS_H
#define __PROX_MULTI_SENSOR_DEFS_H

#include <termios.h>    // POSIX terminal control definitions

//
// Defines for Proximity Multi-Sensor node 
//

#define PROX_MULTI_SENSOR_DEV     "/dev/ttyProxSense"
#define PROX_MULTI_SENSOR_BAUD    38400

#define MAX_PROX_SENSORS     8    // Maximum sensors we can support from the multi-sensor device

#define PROX_SENSOR_MAX_RANGE_MM        2000     // We will assume larger values are not accurate.  
#define PROX_SENSOR_BAD_RANGE_DEFAULT     30     // We will default to short but non-zero range for bogus values

// Default characteristics used in range messages
#define PROX_SENSOR_RADIATION_TYPE         1	// hard code here where this is INFRARED for sensor_msgs/Range.h
#define PROX_SENSOR_FIELD_OF_VIEW_RAD           ((float)(0.44))
#define PROX_SENSOR_MIN_RANGE_METERS            ((float)(0.03))
#define PROX_SENSOR_MAX_RANGE_METERS            ((float)(PROX_SENSOR_MAX_RANGE_MM)/(float)(1000.0))

// The ROS topic we will use to send to the display node
#define  TOPIC_PROX_MULTI_SENSOR_DATA        "prox_multi_sensor_data"

#endif  // __PROX_MULTI_SENSOR_DEFS_H
