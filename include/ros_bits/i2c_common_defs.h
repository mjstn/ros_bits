#ifndef  __I2C_COMMON_DEFS_H
#define  __I2C_COMMON_DEFS_H

//
// IMU node for LSM303 for ROS Node Usage
//
// This file defines the I2C interface and locking semaphore required for atomic access
//

// Defines for the I2C interface.  Note that the sem lock key MUST match 
// for other devices on this I2C or there will be no lock.  
// You may want to have the lock come from some other common system include!

#define  I2C_DEFAULT_DEV                  "/dev/i2c-1"   // device for I2C bus

// If you only have one device on the I2C you may set SEM_PROTECT_I2C to 0
// and the node will initialize it's sem protect for I2C.
// This seems like overkill but allows for cleanest code to just use the sem always.
//
// If  more than one node will use the same I2C dev you must set it to 1
// AND you must have one node in the system like your main control node setup the sem.
#define  SEM_PROTECT_I2C           0            // Set to 1 to use system V sem.

#define  SEM_I2C1_KEY              1111         // For lock of main      I2C bus

#define  I2C_NO_SEM_LOCK           -1           // Used to suppress sem lock on I2C IO calls
#define  I2C_NULL_REG_ADDR         0xff         // Address to suppress secondary chip internal I2C addr

// I2C defines specific for I2C address or the port used in the system
#define  I2C_DEV_FOR_IMU_LSM303           I2C_DEFAULT_DEV
#define  I2C_IMU_LSM303_SEM_KEY           SEM_I2C1_KEY
#define  I2C_IMU_LSM303_REG_ADDR          0x30                 // 7 bit I2C Addr

// Modtronix LCD Display output Node defines
#define  I2C_DEV_FOR_LCD_DISPLAY          I2C_DEFAULT_DEV
#define  I2C_LCD_DISPLAY_SEM_KEY          SEM_I2C1_KEY
#define  I2C_LCD_DISPLAY_REG_ADDR         0x29                 // 7 bit I2C Addr

#endif   // __I2C_COMMON_DEFS_H
