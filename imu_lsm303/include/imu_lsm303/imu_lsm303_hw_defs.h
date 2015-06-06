//
// IMU node for LSM303 for ROS Node Usage
//
// This file defines the I2C interface and locking semaphore required for atomic access
//

// Defines for the I2C interface.  Note that the sem lock key MUST match 
// for other devices on this I2C or there will be no lock.  
// You may want to have the lock come from some other common system include!
#define  SEM_I2C1_KEY              1111         // For lock of main      I2C bus

#define  I2C_NO_SEM_LOCK           -1           // Used to suppress sem lock on I2C IO calls
#define  I2C_NULL_REG_ADDR         0xff         // Address to suppress secondary chip internal I2C addr

// I2C defines specific for I2C address or the port used in the system
#define  I2C_DEV_FOR_IMU_LSM303           "/dev/i2c-1"
#define  I2C_IMU_LSM303_SEM_KEY           I2C_DEFAULT_SEM_KEY
#define  I2C_IMU_LSM303_REG_ADDR          0x30                 // 7 bit I2C Addr

