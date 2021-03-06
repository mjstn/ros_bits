cmake_minimum_required(VERSION 2.8.3)
project(ros_bits)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#  Was Default   add_message_files( FILES Message1.msg Message2.msg)
#  Example       add_message_files(DIRECTORY msg FILES Num.msg)
add_message_files(DIRECTORY msg FILES WheelControlMsg.msg ImuLsm303Msg.msg ProxMultiSensorMsg.msg LcdModtronixMsg.msg CharDisplayMsg.msg)

## Generate services in the 'srv' folder
#  Was Default   add_service_files( FILES Service1.srv Service2.srv)
#  Tutorial      add_service_files(DIRECTORY srv FILES PiBotAddTwoInts.srv)
add_service_files(DIRECTORY srv FILES ServoCtrlSrv.srv)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES ros_bits
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}, 
)

## Specify library search paths and note we make pixy lib external to catkin
#link_directories(
#  ${catkin_LIBRARIES},
#  ~/ros_catkin_ws/install_isolated/lib
#)

## Declare a cpp library
# add_library(ros_bits
#   src/${PROJECT_NAME}/ros_bits.cpp
# )

## Declare a cpp executable
# add_executable(my_node src/my_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
# target_link_libraries(ros_bits_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ros_bits ros_bits_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_bits.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


## Declare a catkin package   (This was in example but no 'stub' appears in file above??
## catkin_package()


## Build nodes for this bot
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wheel_control src/wheel_control.cpp)
target_link_libraries(wheel_control ${catkin_LIBRARIES})
add_dependencies(wheel_control ros_bits_generate_messages_cpp)

add_executable(imu_lsm303 src/imu_lsm303.cpp)
target_link_libraries(imu_lsm303 ${catkin_LIBRARIES})
add_dependencies(imu_lsm303 ros_bits_generate_messages_cpp)

add_executable(prox_multi_sensor src/prox_multi_sensor.cpp)
target_link_libraries(prox_multi_sensor ${catkin_LIBRARIES})
add_dependencies(prox_multi_sensor ros_bits_generate_messages_cpp)

add_executable(lcd_modtronix src/lcd_modtronix.cpp)
target_link_libraries(lcd_modtronix ${catkin_LIBRARIES})
add_dependencies(lcd_modtronix ros_bits_generate_messages_cpp)

add_executable(char_display src/char_display.cpp)
target_link_libraries(char_display ${catkin_LIBRARIES})
add_dependencies(char_display ros_bits_generate_messages_cpp)

add_executable(servo_ctrl_server src/servo_ctrl_server.cpp)
target_link_libraries(servo_ctrl_server ${catkin_LIBRARIES})
add_dependencies(servo_ctrl_server ros_bits_gencpp)
