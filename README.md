# ros_bits   Modules by mark-toys for reuse

This code is meant to help people get started with usage of some nodes and a service that may be of value to other people.   These modules are in use in some of my robots seen at www.mark-toys.com.

I have also taken the liberty of putting a GPIO class here that was formed and put out for usage for Raspberry Pi GPIO control.   Thanks to Hussam Al-Hertani for his repository here:  https://github.com/halherta/RaspberryPi-GPIOClass-v2


The Modules that are in this ros_bits repository are for use in ROS based systems be they robots or other devices so a prerequisite is that the system be running ROS Indigo.  

The I2C modules use a System V semaphore to hold the bus while doing access to devices.  You must initialize this semaphore in your main ROS node to use more than one I2C device.  I had been working on a mode to turn off semaphores if only one node is used but that code is NOT ready so don't think that changing SEM_PROTECT_I2C will work at all!

Your code must run as root OR the user must have full root permissions to use semaphores and hardware.

The names for the I2C bus to use are in i2c_common_defs.h

The name of the serial port for Pololu servo control service is in serial_common_defs.h

The  

Modules:

i2c              Although not a ROS node, I have some I2C code in the include folder.  This is generally pulled in using include statements and not a library so it's easy to use.   This code can be seen in the ROS nodes I have presented in this repository.  

lcd_modtronix    This runs a SparkFun 16x2 line LCD module over I2C.  By default a system V semaphore is expected to be initialized in your main code as discussed in prerequisites.   This node subscribes to a ros topic called /lcd_display_output  which has messages in a custom format of ros_bits/LcdModtronixMsg

imu_lsm303       This is a ROS node that polls the AdaFruit Lcm303 compass/accellerometer and publishes to a ROS topic called /imu_lsm303_data   Your other ROS nodes can then subscribe and pull off items in the message.  By default a system V semaphore is expected to be initialized in your main code as discussed in prerequisites. 

servo_ctrl_server   This is a ROS service that controls a Pololu Mini Serial Servo Controller.   This expects to own a single serial port for output to the servo controller.   This service also has an option that is ON by default that will try to use Raspberry PI GPIO lines using GPIOClass code so if you dont have this be sure to undef SERVO_RESET_LINE in servo_ctrl_defs.h
