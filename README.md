# ros_bits   Modules by mark-toys for reuse

This code is meant to help people get started with usage of some nodes and a service that may be of value to other people.   These modules are in use in some of my robots seen at www.mark-toys.com.

You must have a valid ROS environment with catkin_ws/src folder for this proceedure to work and make.   You can see the general process off of ROS.org but it is assumed you are familiear with the catkin environment and building of ROS code.   Here are the steps in brief if you do have the full make environment already setup in ~/catkin_ws

    cd ~/catkin_ws/src
    git clone https://github.com/mjstn/ros_bits
    cd ~/catkin_ws
    catkin-make


I have also taken the liberty of putting a GPIO class here that was formed and put out for usage for Raspberry Pi GPIO control.   Thanks to Hussam Al-Hertani for his repository here:  https://github.com/halherta/RaspberryPi-GPIOClass-v2


The Modules that are in this ros_bits repository are for use in ROS based systems be they robots or other devices so a prerequisite is that the system be running ROS Indigo.  

    Code          Intent
    I2C           Some Raspberry Pi I2C code that has been used and supports cross-process System V semaphore 
                  to hold the bus while doing access to devices.  You must initialize this semaphore in your 
                  main ROS node to use more than one I2C device.  I had been working on a mode to turn off 
                  semaphores if only one node is used but that code is NOT ready so don't think that changing SEM_PROTECT_I2C will work at all!
                  Your code must run as root OR the user must have full root permissions to use semaphores and hardware.
                  The names for the I2C bus to use are in i2c_common_defs.h

The name of the serial port for Pololu servo control service is in serial_common_defs.h

The  

Modules:

    i2c             Although not a ROS node, I have some I2C code in the include folder.  
                    This is generally pulled in using include statements and not a library so it's easy to use.   
                    This code can be seen in the ROS nodes I have presented in this repository.  

    prox_multi_sensor    This is a node that talks to a Mark-Toys arudino Nano that runs a skitch that 
                    initializes and polls many IR proximity sensors that are ST VL53L0X units.
                    This node currently talks only over a serial port to the Mark-Toys proximity sensor Arduino Nano
                    My intent is to publish the Arduino code in the arudino repository for my shared code.

    char_display    This runs a Parallax #27977-RT serial 5V LCD 16x2 line. This node subscribes to a ros topic 
                    called /char_display_output which was added when this node was coded.  
                    The message format supports updates of fixed number of characters at a given row and column 
                    so sub-fields of the display can be updated. For legacy purposes this node also accepts display 
                    input requests using same messages but legacy topic name of lcd_display_output.  
                    The dual topics for input are intended so that that I can use this more available serial display 
                    in place of the I2C one which is VERY hard to find nowdays.  

    lcd_modtronix   This runs a SparkFun 16x2 line LCD module over I2C.  The display is a Modtronics LCD3S device. 
                    By default a system V semaphore is expected to be initialized in your main code as discussed in prerequisites. 
                    The mesage format supports updates of fixed number of characters at a given row and column 
                    so sub-fields of the display can be updated.   This node subscribes to a ros topic called /lcd_display_output
                    which has messages in a custom format of ros_bits/LcdModtronixMsg

    imu_lsm303      This is a ROS node that polls the AdaFruit Lcm303 compass/accellerometer and publishes to a 
                    ROS topic called /imu_lsm303_data   Your other ROS nodes can then subscribe and pull off items in the message. 
                    By default a system V semaphore is expected to be initialized in your main code as discussed in prerequisites. 

    RoboClaw        The .cpp and .h files for a robo-claw serial packet mode driver I use to run RoboClaw rev 4.1.19.
                    These are just source files so must be linked together with some control code.
                    I have a node called base_control that is still private but I hope to release it here someday.

    wheel_control   A depricated node that will provide differential drive of 2 motors and wheel encoder readback through 
                    the use of the Mark-Toys motor controller and QEI encoder board 
                    (that also controls servos depending on firmware) but this node will not do servos.

servo_ctrl_server   This is a ROS service that controls your servos.  We support the Pololu Mini Serial Servo Controller and as of Sept 2015 we now support Mark-Toys.com servo controller over I2C.   This expects to own a single serial port for output to the servo controller for Pololu but can share the I2C bus with other modules in this repository using the Mark-Toys method of system V semaphores across ROS nodes.   This service also has an option that is ON by default that will try to use Raspberry PI GPIO lines using GPIOClass code so if you dont have this be sure to undef SERVO_RESET_LINE in servo_ctrl_defs.h


# ros_bits   General Purpose Tools

    testPiGpio.py   This handy python tool allows verification of Raspberry Pi 2,3 and PiZero GPIO lines to check if a line has been damaged.   
                    A standard execute of this python script allows the check as long as the user has ability to manipulate GPIO lines.
                    To run it it is required to hook GPIO lines together typically with a female socket that hard wires lines per the script.
                    Look early in the script for arrays where test1output drive test1inputs and make a connector to match that mapping.
                    As of this writing the test2 intput and outputs need to match so the connector can be the same (of course).

