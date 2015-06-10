//
// i2c_utils   This is a set of inline compiled calls that can be used by modules
//
// 20150208    mjstn  added i2c_BufferWrite for multi-byte writes from a reg addr

// i2c_ByteWrite()    Write one byte to a register in a device on the I2C bus
//                    This has a fixed interface defined by Lsm303 driver code
//
//                    We write to a specific I2C device and then write out
//                    a register address per the Lsm303 chip. Finally write 1 data byte
//
// The I2C address is the 8-bit address which is the 7-bit addr shifted left in some code
// If semId is > 0 we lock that sem ID else we run unlocked.
// A non-zero return indicates the write failed.
//
// NOTE: The i2cAddr will be shifted right one bit to use as 7-bit I2C addr
//
static int i2c_ByteWrite(const char *i2cDevFile, int semId, 
                uint8_t i2cAddr, uint8_t* pBuffer, uint8_t WriteAddr)
{
    int retCode = 0;

    if ((semId >= 0) && (ipc_sem_lock(semId) < 0)) {
      // printf("i2c_ByteWrite: Cannot obtain I2C lock!  ERROR: %s\n", strerror(errno));
      return -1;
    }

    // we are now free to access the I2C hardware
    int fd;                                         // File descrition
    int  address   = i2cAddr >> 1;                  // Address of the I2C device
    unsigned char buf[8];                           // Buffer for data being written to the i2c device

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {        // Open port for reading and writing
      retCode = -2;
      // printf("i2c_ByteWrite: Failed to open I2c device!  ERROR: %s\n", strerror(errno));
      goto exitWithSemUnlock;
    }

    // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    if (ioctl(fd, I2C_SLAVE, address) < 0) {        // Set the port options and addr of the dev
      retCode = -3;
      // printf("i2c_ByteWrite: Failed to get bus access to I2c port!  ERROR: %s\n", THIS_NODE_NAME,  strerror(errno));
      goto exitWithFileCloseAndSemUnlock;
    }

    buf[0] = WriteAddr;                         // Internal chip register address
    buf[1] = pBuffer[0];
    if ((write(fd, buf, 2)) != 2) {             // Write both bytes to the i2c port
      // printf("i2c_ByteWrite: Failed to write 0x%x to I2c addr 0x%x [0x%x] register 0x%x!  ERROR: %s\n", 
      //       buf[1], i2cAddr, address, buf[0], strerror(errno));
      retCode = -9;
      goto exitWithFileCloseAndSemUnlock;
    }

  exitWithFileCloseAndSemUnlock:
    close(fd);

  exitWithSemUnlock:
    if ((semId >= 0) && (ipc_sem_unlock(semId) < 0)) {
      retCode = -8;
      //printf("i2c_ByteWrite: Cannot release I2C lock!  ERROR: %s\n", strerror(errno));
    }

  return retCode;
}


// i2c_BufferWrite()  Write one or more bytes to a device on the I2C bus
//
// The I2C address is the 8-bit address which is the 7-bit addr shifted left in some code
// If chipRegAddr != I2C_NULL_REG_ADDR we first do write for the internal chip address for the following read
// If semId is >= 0 we lock that sem ID else we run unlocked.
//
// Returns number of bytes read where 0 or less implies some form of failure
//
// NOTE: The i2cAddr will be shifted right one bit to use as 7-bit I2C addr
//
#define I2C_MAX_BYTES_TO_WRITE  128
static int i2c_BufferWrite(const char *i2cDevFile, int semId, 
                uint8_t i2cAddr, uint8_t* pBuffer, uint8_t chipRegAddr, uint16_t NumByteToWrite)
{
   int retCode      = 0;

    if (NumByteToWrite <= 0) {
      return 0;		// Not illegal and we are not going to hold the users hand on this silly error
    }
    if (NumByteToWrite > I2C_MAX_BYTES_TO_WRITE) {
      // printf("i2c_BufferWrite: Cannot support over %d bytes\n", I2C_MAX_BYTES_TO_WRITE);
      return -5;
    }

    if ((semId >= 0) && (ipc_sem_lock(semId) < 0)) {
      // printf("i2c_BufferWrite: Cannot obtain I2C lock!  ERROR: %s\n", strerror(errno));
      return -1;
    }

    // we are now free to access the I2C hardware
    int fd;                                         // File descrition
    int  address   = i2cAddr >> 1;                  // Address of the I2C device
    unsigned char buf[256];                         // Buffer for data being written to the i2c device
    int bytesWritten = 0;
    int caOfst = 0;

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {        // Open port for reading and writing
      retCode = -2;
      // printf("i2c_BufferWrite: Failed to open I2c device!  ERROR: %s\n", strerror(errno));
      goto exitWithSemUnlock;
    }

    // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    if (ioctl(fd, I2C_SLAVE, address) < 0) {        // Set the port options and addr of the dev
      retCode = -3;
      // printf("i2c_BufferWrite: Failed to get bus access to I2c port!  ERROR: %s\n", THIS_NODE_NAME,  strerror(errno));
      goto exitWithFileCloseAndSemUnlock;
    }

    if (chipRegAddr != I2C_NULL_REG_ADDR) {     // Suppress reg address if special 'null' value was used
      buf[0] = chipRegAddr;                     // Internal chip register address
      caOfst = 1;
    }

    // Now we have to write to the chip into incrementing addresses for each byte written
    for (int i=0; i<NumByteToWrite ; i++) {
      buf[caOfst+i] = pBuffer[i];
    }
    bytesWritten = write(fd, buf, (caOfst+NumByteToWrite));
    if (bytesWritten != (caOfst+NumByteToWrite)) {   // Write all the bytes of data in one call
      // printf("i2c_ByteWrite: Failed to write %d bytes to I2c addr 0x%x [0x%x] from reg register 0x%x!  ERROR: %s\n",
      //       NumByteToWrite, i2cAddr, address, buf[0], strerror(errno));
      retCode = -9;
      goto exitWithFileCloseAndSemUnlock;
    }

    retCode = bytesWritten - caOfst;

  exitWithFileCloseAndSemUnlock:
    close(fd);

  exitWithSemUnlock:
    if ((semId >= 0) && (ipc_sem_unlock(semId) < 0)) {
      retCode = -8;
      //printf("i2c_ByteWrite: Cannot release I2C lock!  ERROR: %s\n", strerror(errno));
    }

  return retCode;
}

// i2c_BufferRead()   Our system specific hook for Lsm303 driver code with ros debug
//                    This has a fixed interface defined by Lsm303 driver code
//
// The I2C address is the 8-bit address which is the 7-bit addr shifted left in some code
// If chipRegAddr != I2C_NULL_REG_ADDR we first do write for the internal chip address for the following read
// If semId is > 0 we lock that sem ID else we run unlocked.
//
// Returns number of bytes read where 0 or less implies some form of failure
//
// NOTE: The i2cAddr will be shifted right one bit to use as 7-bit I2C addr
//
static int i2c_BufferRead(const char *i2cDevFile, int semId, 
                uint8_t i2cAddr, uint8_t* pBuffer, uint8_t chipRegAddr, uint16_t NumByteToRead)
{
   int bytesRead = 0;
   int retCode   = 0;

    if ((semId >= 0) && (ipc_sem_lock(semId) < 0)) {
      // printf("i2c_BufferRead: Cannot obtain I2C lock!  ERROR: %s\n", strerror(errno));
      return -1;
    }

    // we are now free to access the I2C hardware
    int fd;                                         // File descrition
    int  address   = i2cAddr >> 1;                  // Address of the I2C device
    unsigned char buf[8];                           // Buffer for data being written to the i2c device

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {        // Open port for reading and writing
      retCode = -2;
      // printf("i2c_BufferRead: Failed to open I2c device!  ERROR: %s\n", strerror(errno));
      goto exitWithSemUnlock;
    }

    // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    if (ioctl(fd, I2C_SLAVE, address) < 0) {        // Set the port options and addr of the dev
      retCode = -3;
      // printf("i2c_BufferRead: Failed to get bus access to I2c port!  ERROR: %s\n", THIS_NODE_NAME,  strerror(errno));
      goto exitWithFileCloseAndSemUnlock;
    }

    if (chipRegAddr != I2C_NULL_REG_ADDR) {     // Suppress reg address if special 'null' value was used
      buf[0] = chipRegAddr;                     // Internal chip register address
      if ((write(fd, buf, 1)) != 1) {           // Write both bytes to the i2c port
        retCode = -4;
        // printf("i2c_BufferRead: Failed to write chip register 0x%x to I2c addr 0x%x [0x%x]!  ERROR: %s\n",
        //     buf[0], i2cAddr, address, strerror(errno));
        goto exitWithFileCloseAndSemUnlock;
      }
    }

    // Now we have to read from the chip as the register start address was just setup
    // if (ioctl(fd, I2C_SLAVE, address) < 0) {        // Set the port options and addr of the dev
    //   ROS_INFO("%s: Failed to get bus access to I2c port!  ERROR: %s\n", THIS_NODE_NAME,  strerror(errno));
    //   goto exitWithFileCloseAndSemUnlock;
    // }

    bytesRead = read(fd, pBuffer, NumByteToRead);
    if (bytesRead != NumByteToRead) {      // verify the number of bytes we requested were read
      // printf("i2c_BufferRead: Failed reading %d bytes from I2c addr 0x%x [0x%x]!  ERROR: %s\n",
      //          NumByteToRead, i2cAddr, address, strerror(errno));
      retCode = -9;
      goto exitWithFileCloseAndSemUnlock;
    }
    retCode = bytesRead;

  exitWithFileCloseAndSemUnlock:
    close(fd);

  exitWithSemUnlock:
    if ((semId >= 0) && (ipc_sem_unlock(semId) < 0)) {
      retCode = -8;
      //printf("i2c_ByteWrite: Cannot release I2C lock!  ERROR: %s\n", strerror(errno));
    }

  return retCode;
}


