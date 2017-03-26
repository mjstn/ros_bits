#include "RoboClaw.h"

#include <vector>
#include <ros/ros.h>

#include <termios.h>    // POSIX terminal control definitions

using namespace std;

#define SetDwordInByteArray(arg,array,base) array[base]=(uint8_t)((arg>>24)&0xff);array[base+1]=(uint8_t)((arg>>16)&0xff);array[base+2]=(uint8_t)((arg>>8)&0xff);array[base+3]=((uint8_t)arg&0xff)
#define SetInt16InByteArray(arg,array,base) array[base]=(uint8_t)((arg>>8)&0xff);array[base+1]=(uint8_t)(arg&0xff)
#define SetDWORDval(arg) (uint8_t)((arg>>24)&0xff),(uint8_t)((arg>>16)&0xff),(uint8_t)((arg>>8)&0xff),((uint8_t)arg&0xff)
#define SetWORDval(arg) (uint8_t)((arg>>8)&0xff),(uint8_t)(arg&0xff)

// -------------------  Start Original USB serial COMM driver -----------------

//! Macros for throwing an exception with a message, passing args
#define USB_SERIAL_EXCEPT(msg, ...)                                         \
  {                                                                     \
    char buf[1000];                                                     \
    snprintf(buf, 1000, msg " (in USBSerial::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw USBSerial::Exception(buf);                                    \
  }

void USBSerial::Open(const char *port) {
  if (IsOpen()) {
    Close();
  }

  fd_ = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);
  buf_start_ = buf_end_ = 0;
  if (fd_ == -1) {
    USB_SERIAL_EXCEPT("Failed to open port: %s. %s (errno = %d)",
                  port, strerror(errno), errno);
  }

  try {
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    fl.l_pid = getpid();

    if (fcntl(fd_, F_SETLK, &fl) != 0) {
      USB_SERIAL_EXCEPT("Device %s is already locked", port);
    }

    struct termios newtio;
    tcgetattr(fd_, &newtio);
    memset(&newtio.c_cc, 0, sizeof(newtio.c_cc)); // Clear special characters
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newtio) < 0) {
      USB_SERIAL_EXCEPT("Couldn't set serial port attributes: %s", port);
    }

    usleep(200000);
    Flush();
  } catch (USBSerial::Exception &e) {
    if (fd_ != -1) {
      close(fd_);
    }
    fd_ = -1;
    throw e;
  }
}

bool USBSerial::IsOpen() {
  return fd_ != -1;
}

void USBSerial::Close() {
  if (IsOpen()) {
    try {
      Flush();
    } catch (USBSerial::Exception &e) {
      ROS_WARN("While closing, error while flushing=%s", e.what());
    }
  }
  int rv = close(fd_);
  if (rv != 0) {
    USB_SERIAL_EXCEPT("Error closing port: %s (%d)", strerror(errno), errno);
  }
}

int USBSerial::Flush() {
  int retval = tcflush(fd_, TCIOFLUSH);
  if (retval != 0)
    USB_SERIAL_EXCEPT("tcflush failed %i", retval);
  buf_start_ = 0;
  buf_end_ = 0;
  return retval;
}

int USBSerial::Write(const unsigned char *data, int len) {
  // TODO: Because this is synchronous, if the device behaves weirdly we can
  // hang here
  int origflags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK);
  // fprintf(stderr, "Writing ");
  // for (int i = 0; i < len; ++i) {
  //   fprintf(stderr, "%02x ", data[i]);
  // }
  // fprintf(stderr, "\n");
  ssize_t retval = write(fd_, data, len);
  int fputserrno = errno;
  fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);
  errno = fputserrno;
  if (retval != -1 ) {
    return retval;
  } else {
    USB_SERIAL_EXCEPT("write() failed: %s (%d)", strerror(errno), errno);
  }
}

int USBSerial::Read(char *buf, int len, int timeout, bool translate) {
  int current = 0;
  struct pollfd ufd[1];
  int retval;
  ufd[0].fd = fd_;
  ufd[0].events = POLLIN;

  while (true) {
    if (buf_start_ == buf_end_) {
      if ((retval = poll(ufd, 1, timeout)) < 0) {
        USB_SERIAL_EXCEPT("Poll failed %s (%d)", strerror(errno), errno);
      } else if (retval == 0) {
        USB_SERIAL_EXCEPT("Timeout reached");
      } else if (ufd[0].revents & POLLERR) {
        USB_SERIAL_EXCEPT("Error on socket");
      } else if (ufd[0].revents & POLLIN) {
        int bytes = read(fd_, read_buf_, sizeof(read_buf_));
        buf_start_ = 0;
        buf_end_ = bytes;
        if (buf_end_ == 0) {
          USB_SERIAL_EXCEPT("Read 0 bytes");
        }
      } else {
        USB_SERIAL_EXCEPT("Unhandled case!");
      }
    }

    while (buf_start_ != buf_end_) {
      if (translate) {
        if (current == len - 1) {
          buf[current] = 0;
          USB_SERIAL_EXCEPT("Buffer filled without end of line being found");
        }
        buf[current] = read_buf_[buf_start_];
        buf_start_++;
        if (read_buf_[current++] == '\n') {
          buf[current] = 0;
          return current;
        }
      } else {
        buf[current] = read_buf_[buf_start_];
        buf_start_++;
        current++;
        if (current == len) {
          return current;
        }
      }
    }
  }
}


// --------------------------  Start Packet serial COMM driver -----------------

//! Macros for throwing an exception with a message, passing args
#define PACKET_SERIAL_EXCEPT(msg, ...)                                         \
  {                                                                     \
    char buf[1000];                                                     \
    snprintf(buf, 1000, msg " (in PACKETSerial::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw PACKETSerial::Exception(buf);                                    \
  }

PACKETSerial::PACKETSerial (uint8_t address)
{
	address_ = address;
	fd_ = -1;
}

uint8_t PACKETSerial::GetAddress() {
	return address_;
}

// Open serial dev and set baud rate
void PACKETSerial::Open(const char *serDev, speed_t baudRate) {
  if (IsOpen()) {
    Close();
  }

  fd_ = open(serDev, (O_RDWR | O_NONBLOCK | O_NDELAY));  // | O_NOCTTY);
  buf_start_ = buf_end_ = 0;
  if (fd_ == -1) {
    PACKET_SERIAL_EXCEPT("Failed to open port: %s. %s (errno = %d)",
                  serDev, strerror(errno), errno);
  }

  try {
    
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( fd_, &tty ) != 0 ) {
    	PACKET_SERIAL_EXCEPT("Failed to get port attributes!");
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, baudRate);
    cfsetispeed (&tty, baudRate);

    /* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush( fd_, TCIFLUSH );
	if ( tcsetattr ( fd_, TCSANOW, &tty ) != 0) {
    	PACKET_SERIAL_EXCEPT("Failed to set port attributes!");
	}

    // TODO: Ping device and see if we can talk to it

    usleep(10000);
    Flush();
  } catch (PACKETSerial::Exception &e) {
    if (fd_ != -1) {
      close(fd_);
    }
    fd_ = -1;
    throw e;
  }
}

bool PACKETSerial::IsOpen() {
  return fd_ != -1;
}

void PACKETSerial::Close() {
  if (IsOpen()) {
    try {
      Flush();
    } catch (PACKETSerial::Exception &e) {
      ROS_WARN("While closing, error while flushing=%s", e.what());
    }
  }
  int rv = close(fd_);
  if (rv != 0) {
    PACKET_SERIAL_EXCEPT("Error closing port: %s (%d)", strerror(errno), errno);
  }
}

void PACKETSerial::Crc_update(uint16_t &crc, uint8_t data)
{
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
}


//  Write:  Send bytes in the buffer. No checksum or waiting for ACK
//
int PACKETSerial::Write(const unsigned char *data, int len) {

  int retval = 0;;
  int bytes;

  if (!IsOpen()) {
    PACKET_SERIAL_EXCEPT("write() failed: Serial device is not open yet! ");
  }

  // Now write out the bytes with no additions
  bytes = len;
  if ((write(fd_, data, bytes)) != bytes) { 
    PACKET_SERIAL_EXCEPT("write() failed: Full write of %d bytes not completed", bytes);
  }

  return 0;
}

//  Write:  Send a packet to RoboClaw and get back the ACK byte or timeout
//  The address in the class is sent then the data.
//  The 1st byte of data should be RoboClaw command then remaining data
//  The CRC includess the address, command and data bytes
//
int PACKETSerial::WritePacket(const unsigned char *data, int len) {

  int retval = 0;;
  uint16_t crc = 0;
  int     i;
  uint8_t  outBuf[PACKET_MAX_LEN+2];
  uint8_t  inBuf[4];
  int      bytes;
  int      ackRetries;

  if (len > PACKET_MAX_LEN) {
    PACKET_SERIAL_EXCEPT("write() failed: %s (%d)", "Write is too long with max bytes being ", PACKET_MAX_LEN);
  }

  if (!IsOpen()) {
    PACKET_SERIAL_EXCEPT("write() failed: Serial device is not open yet! ");
  }

  // Place addr in first byte then data after that and write it all out at once
  memcpy(&outBuf[0], data, len);

  // Calculate crc and stuff at end of data bytes
  crc = 0;
  for (i=0; i<len ; i++) {
      Crc_update(crc, outBuf[i]);
  }
  outBuf[len] = crc >> 8;
  outBuf[(len+1)] = crc & 0xff;

  // Now write out data (usually) addr, cmd with data, the two bytes of crc (msb first)
  bytes = len+2;
  if (Write(&outBuf[0], bytes)) { 
    PACKET_SERIAL_EXCEPT("write() failed: Full write of %d bytes not completed", bytes);
  }

  // Now we wait for ACK from the serial port or we have a fault in the write
  ackRetries = PACKET_RETRIES;
  retval   = -1;
  inBuf[0] = 0xdb;
  do {
      bytes = read(fd_, &inBuf[0], 1);
      if (bytes >= 1) {
          if (inBuf[0] == PACKET_ACK_BYTE) {
              retval = 0;
              continue;		// Got the rexpected ACK byte so all is well
          }
      }
      usleep(PACKET_RETRIES_USEC);
      ackRetries -= 1;
  } while((retval < 0) && (ackRetries > 0)); 

  if ((retval < 0) || (ackRetries <= 0)) {
    PACKET_SERIAL_EXCEPT("WritePacket() failed: Timeout waiting for ACK!");
  }

  return 0;
}

int PACKETSerial::Flush() {
  uint8_t  inByte[4];
  while (1 == read(fd_, &inByte[0], 1)) { }
  return 0;
}

// Read len bytes into buffer or timeout while trying
// Return number of bytes that were read
int PACKETSerial::Read(char *buf, int len, int timeoutMsec) {
  int      retval = 0;
  int      i;
  uint8_t  *inPtr;
  uint8_t  inByte[4];
  int      bytes = 0;
  int      byteRetries;

  inPtr = (uint8_t*)buf;

  byteRetries = timeoutMsec;
  do {
    if (1 == read(fd_, &inByte[0], 1)) {
       // printf("DEBUG: Read got byte 0x%x \n", inByte);
       *inPtr++ = inByte[0];	// Stash another received char in the buffer
       bytes += 1;
       byteRetries = PACKET_PER_BYTE_RETRIES;   // Reset retries for per byte
    } else {
       byteRetries -= 1;
       usleep(PACKET_RETRIES_USEC);
    }
  } while ((bytes < len) && (byteRetries > 0)); 
  
  if ((bytes < len) || (byteRetries <= 0))  {
    PACKET_SERIAL_EXCEPT("read() failed: Timeout waiting for all bytes!");
  }

  return bytes;  
}

int roboclaw_restart_usb() {
  const int16_t ROBOCLAW_PRODUCT = 0x2404;
  const int16_t ROBOCLAW_VENDOR = 0x3eb;

  libusb_init(NULL);
  libusb_device **list;
  ssize_t cnt = libusb_get_device_list(NULL, &list);
  if (cnt < 0) {
    ROS_WARN("Couldn't get device list: %s\n", libusb_error_name(cnt));
    return -1;
  }

  size_t problems = 0;
  int err = 0;
  for (int i = 0; i < cnt; i++) {
    libusb_device *device = list[i];
    struct libusb_device_descriptor desc;

    if (libusb_get_device_descriptor(device, &desc) != 0) {
      ROS_WARN("Couldn't get descriptor: %s", libusb_error_name(cnt));
      continue;
    }

    if (desc.idProduct == ROBOCLAW_PRODUCT &&
        desc.idVendor == ROBOCLAW_VENDOR) {
      libusb_device_handle *handle;
      if ((err = libusb_open(device, &handle)) != 0) {
        ROS_WARN("Couldn't open device: %s\n", libusb_error_name(err));
        problems |= 1;
      } else {
        if ((err = libusb_reset_device(handle)) != 0) {
          ROS_WARN("Couldn't reset device: %s\n", libusb_error_name(err));
          problems |= 1;
        } else {
          libusb_close(handle);
        }
      }
    }
  }
  libusb_free_device_list(list, 1);
  libusb_close(NULL);
  return problems;
}

//
// Constructor
// We are not going to full virtual class members to retain legacy code
//
RoboClaw::RoboClaw(USBSerial *ser) {
  setSerial(ser);
}
RoboClaw::RoboClaw(PACKETSerial *ser) {
  setSerial(ser);
}

//
// Destructor
//
RoboClaw::~RoboClaw() {

}


// Set serial port to be USB serial dev for all IO
void RoboClaw::setSerial(USBSerial *ser) {
  if (ser == NULL || !ser->IsOpen()) {
    throw invalid_argument("RoboClaw needs open USBSerial device");
  }
  usbSer_ = ser;
  connectMode_ = CONNECT_MODE_USB;
}

// Set serial port to be COMM serial dev for all IO
void RoboClaw::setSerial(PACKETSerial *ser) {
  if (ser == NULL || !ser->IsOpen()) {
    throw invalid_argument("RoboClaw needs open COMM Serial device");
  }
  packetSer_ = ser;
  connectMode_ = CONNECT_MODE_PACKET;
}


// write_buff:  Write bytes from an array of unsigned 8-bit values
//
// This call is a cleanup of the unorthodox write_n call from prior driver code.
// We select USB or Serial packet mode based on prior setup to RoboClaw class
//
void RoboClaw::write_buff(uint8_t byteCnt, uint8_t *buff) {
  int ind = 0;
  uint8_t crc=0;

  // send data with crc so get the crc now
  // fprintf(stderr, "Sending: ");
  for(uint8_t index=0; index < byteCnt; index++) {
    uint8_t data = buff[index];
    // fprintf(stderr, "%02x ", data);
    crc += data;
    buff[ind++] = data;
  }
  // fprintf(stderr, "\n");
  buff[ind++] = crc & 0x7F;

  // Use the driver that has been configured for this instance
  if (connectMode_ == CONNECT_MODE_USB) {
      usbSer_->Write(buff, byteCnt + 1);
  } else {
      packetSer_->WritePacket(&buff[0], byteCnt);
  }
}

// write_n generally starts with address, command then data bytes (if any)
//
// WARNING: write_n is DEPRICATED due to it's non-standard 'clever' operation is hard to support
//          One can argue that write_n is clever but it is not straightforward byte packing 
//
// We select USB or Serial packet mode based on prior setup to RoboClaw class
// There is a trick in this method in that each argument will be converted
// to an int then only the lower 8 bits are used.  
// Really should be byte array but that is how it was done
//
void RoboClaw::write_n(uint8_t cnt, ... ) {
  static unsigned char buff[256];
  int ind = 0;
  uint8_t crc=0;

  // send data with crc
  va_list marker;
  va_start(marker, cnt);
  // fprintf(stderr, "Sending: ");
  for(uint8_t index=0; index < cnt; index++) {
    uint8_t data = va_arg(marker, int);
    // fprintf(stderr, "%02x ", data);
    crc += data;
    buff[ind++] = data;
  }
  // fprintf(stderr, "\n");
  va_end(marker);              /* Reset variable arguments.      */
  buff[ind++] = crc & 0x7F;

  // Use the driver that has been configured for this instance
  if (connectMode_ == CONNECT_MODE_USB) {
      usbSer_->Write(buff, cnt + 1);
  } else {
      packetSer_->WritePacket(&buff[0], cnt);
  }
}

void RoboClaw::write(uint8_t byte) {
  // fprintf(stderr, "Sending: 0x%02x\n", byte);

  // Use the driver that has been configured for this instance
  if (connectMode_ == CONNECT_MODE_USB) {
      usbSer_->Write(&byte, 1);
  } else {
      packetSer_->Write(&byte, 1);
  }
}

// Single byte read.  Return the single byte that was read
uint8_t RoboClaw::read() {
  char d[1];
  int rv;

  if (connectMode_ == CONNECT_MODE_USB) {
      rv = usbSer_->Read(d, 1, 100, false);
  } else {
      rv = packetSer_->Read(&d[0], 1, PACKET_PER_BYTE_RETRIES);
  }

  if (rv != 1) {
    // fprintf(stderr, "Rv = %i\n", rv);
    throw std::runtime_error("RoboClaw::read() Didn't get byte from read()");
  } else {
    // fprintf(stderr, "Read 0x%02x\n", d[0]);
  }
  return static_cast<uint8_t>(d[0]);
}

void RoboClaw::ForwardM1(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M1FORWARD;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::BackwardM1(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M1BACKWARD;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = SETMINMB;
  buff[2] = voltage;
  write_buff(3, &buff[0]);
}

void RoboClaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = SETMAXMB;
  buff[2] = voltage;
  write_buff(3, &buff[0]);
}

void RoboClaw::ForwardM2(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M2FORWARD;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::BackwardM2(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M2BACKWARD;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::ForwardBackwardM1(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M17BIT;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::ForwardBackwardM2(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M27BIT;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::ForwardMixed(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDFORWARD;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::BackwardMixed(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDBACKWARD;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::TurnRightMixed(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDRIGHT;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::TurnLeftMixed(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDLEFT;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::ForwardBackwardMixed(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDFB;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

void RoboClaw::LeftRightMixed(uint8_t address, uint8_t speed) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDLR;
  buff[2] = speed;
  write_buff(3, &buff[0]);
}

uint32_t RoboClaw::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid) {
  uint8_t send[2] = {address, cmd};
  if (connectMode_ == CONNECT_MODE_USB) {
      usbSer_->Write(send, 2);
  } else {
      packetSer_->Write(send, 2);
  }

  uint8_t crc = address;
  crc+=cmd;

  uint32_t value;
  uint8_t data = read();
  crc+=data;
  value=(uint32_t)data<<24;

  data = read();
  crc+=data;
  value|=(uint32_t)data<<16;

  data = read();
  crc+=data;
  value|=(uint32_t)data<<8;

  data = read();
  crc+=data;
  value|=(uint32_t)data;

  data = read();
  crc+=data;
  if(status)
    *status = data;
  data = read();
  if(valid)
    *valid = ((crc&0x7F)==data);

  return value;
}

uint32_t RoboClaw::ReadEncM1(uint8_t address, uint8_t *status,bool *valid) {
  if (connectMode_ == CONNECT_MODE_USB) {
      usbSer_->Flush();
  } else {
      packetSer_->Flush();
  }
  return (Read4_1(address,GETM1ENC,status,valid));
}

uint32_t RoboClaw::ReadEncM2(uint8_t address, uint8_t *status,bool *valid) {
  if (connectMode_ == CONNECT_MODE_USB) {
      usbSer_->Flush();
  } else {
      packetSer_->Flush();
  }
  return (Read4_1(address,GETM2ENC,status,valid));
}

void RoboClaw::SetEncM1(uint8_t address, uint32_t value) {
  uint8_t buff[12];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = SETM1ENC;
  SetDwordInByteArray(value,buff,2);
  write_buff(6, &buff[0]);
}
void RoboClaw::SetEncM2(uint8_t address, uint32_t value) {
  uint8_t buff[12];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = SETM2ENC;
  SetDwordInByteArray(value,buff,2);
  write_buff(6, &buff[0]);
}

int32_t RoboClaw::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid) {
  return Read4_1(address,GETM1SPEED,status,valid);
}

int32_t RoboClaw::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid) {
  return Read4_1(address,GETM2SPEED,status,valid);
}

void RoboClaw::ResetEncoders(uint8_t address) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = RESETENC;
  write_buff(2, &buff[0]);
}

bool RoboClaw::ReadVersion(uint8_t address, string *version) {
  version->resize(32);

  uint8_t crc;
  write(address);
  crc=address;
  write(GETVERSION);
  crc+=GETVERSION;

  for(uint8_t i=0;i<32;i++) {
    (*version)[i] = read();
    crc += (*version)[i];
    if ((*version)[i] == 0) {
      return (crc&0x7F) == read();
    }
  }
  return false;
}

uint16_t RoboClaw::Read2(uint8_t address,uint8_t cmd,bool *valid) {
  uint8_t crc;
  write(address);
  crc=address;
  write(cmd);
  crc+=cmd;

  uint16_t value;
  uint8_t data = read();
  crc+=data;
  value=(uint16_t)data<<8;

  data = read();
  crc+=data;
  value|=(uint16_t)data;

  data = read();
  if(valid)
    *valid = ((crc&0x7F)==data);

  return value;
}

uint16_t RoboClaw::ReadMainBatteryVoltage(uint8_t address,bool *valid) {
  return Read2(address,GETMBATT,valid);
}

uint16_t RoboClaw::ReadLogicBattVoltage(uint8_t address,bool *valid) {
  return Read2(address,GETLBATT,valid);
}

void RoboClaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = SETMINLB;
  buff[2] = voltage;
  write_buff(3, &buff[0]);
}

void RoboClaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage) {
  uint8_t buff[8];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = SETMAXLB;
  buff[2] = voltage;
  write_buff(3, &buff[0]);
}


void RoboClaw::SetM1Constants(uint8_t address, uint32_t kd, uint32_t kp, uint32_t ki, uint32_t qpps) {
  uint8_t buff[24];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = SETM1PID;
  SetDwordInByteArray(kd,buff,2);
  SetDwordInByteArray(kp,buff,6);
  SetDwordInByteArray(ki,buff,10);
  SetDwordInByteArray(qpps,buff,14);
  write_buff(18, &buff[0]);
}

void RoboClaw::SetM2Constants(uint8_t address, uint32_t kd, uint32_t kp, uint32_t ki, uint32_t qpps) {
  uint8_t buff[24];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = SETM2PID;
  SetDwordInByteArray(kd,buff,2);
  SetDwordInByteArray(kp,buff,6);
  SetDwordInByteArray(ki,buff,10);
  SetDwordInByteArray(qpps,buff,14);
  write_buff(18, &buff[0]);
}

uint32_t RoboClaw::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid) {
  return Read4_1(address,GETM1ISPEED,status,valid);
}

uint32_t RoboClaw::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid) {
  return Read4_1(address,GETM2ISPEED,status,valid);
}

void RoboClaw::DutyM1(uint8_t address, uint16_t duty) {
  uint8_t buff[16];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M1DUTY;
  SetInt16InByteArray(duty,buff,2);
  write_buff(4, &buff[0]);
}

void RoboClaw::DutyM2(uint8_t address, uint16_t duty) {
  uint8_t buff[16];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M2DUTY;
  SetInt16InByteArray(duty,buff,2);
  write_buff(4, &buff[0]);
}

void RoboClaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2) {
  uint8_t buff[16];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDDUTY;
  SetInt16InByteArray(duty1,buff,2);
  SetInt16InByteArray(duty1,buff,4);
  write_buff(6, &buff[0]);
}

void RoboClaw::SpeedM1(uint8_t address, uint32_t speed) {
  uint8_t buff[16];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M1SPEED;
  SetDwordInByteArray(speed,buff,2);
  write_buff(6, &buff[0]);
}

void RoboClaw::SpeedM2(uint8_t address, uint32_t speed) {
  uint8_t buff[16];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M2SPEED;
  SetDwordInByteArray(speed,buff,2);
  write_buff(6, &buff[0]);
}

void RoboClaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2) {
  uint8_t buff[16];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDSPEED;
  SetDwordInByteArray(speed1,buff,2);
  SetDwordInByteArray(speed2,buff,6);
  write_buff(10, &buff[0]);
}

void RoboClaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed) {
  uint8_t buff[16];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M1SPEEDACCEL;
  SetDwordInByteArray(accel,buff,2);
  SetDwordInByteArray(speed,buff,6);
  write_buff(10, &buff[0]);
}

void RoboClaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed) {
  uint8_t buff[16];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = M2SPEEDACCEL;
  SetDwordInByteArray(accel,buff,2);
  SetDwordInByteArray(speed,buff,6);
  write_buff(10, &buff[0]);
}

void RoboClaw::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2) {
  uint8_t buff[20];    // must include space at end for 2 byte crc
  buff[0] = address;
  buff[1] = MIXEDSPEEDACCEL;
  SetDwordInByteArray(accel,buff,2);
  SetDwordInByteArray(speed1,buff,6);
  SetDwordInByteArray(speed2,buff,10);
  write_buff(14, &buff[0]);
}

void RoboClaw::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag) {
  write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

void RoboClaw::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag) {
  write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

void RoboClaw::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag) {
  write_n(19,address,M1SPEEDDIST,SetDWORDval(speed2),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

void RoboClaw::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag) {
  write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

void RoboClaw::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag) {
  write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

void RoboClaw::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag) {
  write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2) {
  bool valid;
  uint16_t value = Read2(address,GETBUFFERS,&valid);
  if(valid) {
    depth1 = value>>8;
    depth2 = value;
  }
  return valid;
}

uint32_t RoboClaw::Read4(uint8_t address, uint8_t cmd, bool *valid) {
  uint8_t send[2] = {address, cmd};
  if (connectMode_ == CONNECT_MODE_USB) {
      usbSer_->Write(send, 1);
  } else {
      packetSer_->Write(send, 1);
  }

  uint8_t crc = address;
  crc+=cmd;

  uint32_t value;
  uint8_t data = read();
  crc+=data;
  value=(uint32_t)data<<24;

  data = read();
  crc+=data;
  value|=(uint32_t)data<<16;

  data = read();
  crc+=data;
  value|=(uint32_t)data<<8;

  data = read();
  crc+=data;
  value|=(uint32_t)data;

  data = read();
  if(valid)
    *valid = ((crc&0x7F)==data);

  return value;
}

bool RoboClaw::ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2) {
  bool valid;
  uint32_t value = Read4(address,GETCURRENTS,&valid);
  if(valid) {
    current1 = value>>16;
    current2 = value;
  }
  return valid;
}

void RoboClaw::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2) {
  write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

void RoboClaw::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag) {
  write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

void RoboClaw::DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel) {
  write_n(6,address,M1DUTY,SetWORDval(duty),SetWORDval(accel));
}

void RoboClaw::DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel) {
  write_n(6,address,M2DUTY,SetWORDval(duty),SetWORDval(accel));
}

void RoboClaw::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint16_t accel1, uint16_t duty2, uint16_t accel2) {
  write_n(10,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(accel1),SetWORDval(duty2),SetWORDval(accel2));
}

uint32_t RoboClaw::Read_uint32(uint8_t &crc) {
  uint32_t value;
  uint8_t data = read();
  crc+=data;
  value=(uint32_t)data<<24;

  data = read();
  crc+=data;
  value|=(uint32_t)data<<16;

  data = read();
  crc+=data;
  value|=(uint32_t)data<<8;

  data = read();
  crc+=data;
  value|=(uint32_t)data;
  
  return value;
}

bool RoboClaw::ReadPIDM1(uint8_t address, uint32_t &p, uint32_t &i, uint32_t &d, uint32_t &qpps) {
  uint8_t crc;
  write(address);
  crc=address;
  write(GETM1PID);
  crc+=GETM1PID;

  p = Read_uint32(crc);
  i = Read_uint32(crc);
  d = Read_uint32(crc);
  qpps = Read_uint32(crc);

  return ((crc&0x7F)==read());
}

bool RoboClaw::ReadPIDM2(uint8_t address, uint32_t &p, uint32_t &i, uint32_t &d, uint32_t &qpps) {
  uint8_t crc;
  write(address);
  crc=address;
  write(GETM2PID);
  crc+=GETM2PID;

  p = Read_uint32(crc);
  i = Read_uint32(crc);
  d = Read_uint32(crc);
  qpps = Read_uint32(crc);

  return ((crc&0x7F)==read());
}

uint8_t RoboClaw::ReadError(uint8_t address,bool *valid) {
  uint8_t crc;
  write(address);
  crc=address;
  write(GETERROR);
  crc+=GETERROR;

  uint8_t value = read();
  crc+=value;

  if(valid)
    *valid = ((crc&0x7F)==read());
  else
    read();

  return value;
}

void RoboClaw::WriteNVM(uint8_t address) {
  write_n(2,address,WRITENVM);
}

void RoboClaw::SetPWM(uint8_t address, uint8_t resolution) {
  write_n(3, address, SETPWM, resolution);
}
