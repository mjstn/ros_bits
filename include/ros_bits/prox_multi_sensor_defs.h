#ifndef __PROX_MULTI_SENSOR_DEFS_H
#define __PROX_MULTI_SENSOR_DEFS_H

//
// Defines for Proximity Multi-Sensor node 
//

#define PROX_MULTI_SENSOR_DEV     "/dev/ttyProxMultiSensor"

#define MAX_PROX_SENSORS     8    // Maximum sensors we can support from the multi-sensor device

// Default characteristics used in range messages
#define PROX_SENSOR_RADIATION_TYPE         1	// hard code here where this is INFRARED for sensor_msgs/Range.h
#define PROX_SENSOR_FIELD_OF_VIEW_RAD           ((float)(0.44))
#define PROX_SENSOR_MIN_RANGE_METERS            ((float)(0.03))
#define PROX_SENSOR_MAX_RANGE_METERS            ((float)(1.20))

// The ROS topic we will use to send to the display node
#define  TOPIC_PROX_MULTI_SENSOR_DATA        "prox_multi_sensor_data"

#endif  // __PROX_MULTI_SENSOR_DEFS_H
