# droidBot launch files with clues as to forming bot with Lidar Nav

These launch files are taken from a working robot called DroidBot from Mark-Toys.com

These launch files are meant to offer clues to persons putting together a robot that has a Neato XV-11 lidar.
This set of launch files will NOT make a working system.  These files only point to the sort of things required.
Many common ROS components are assumed that are part of ROS navigation.
Many parts are specific to my robot so they are not given either.  These files just offer clues as to pieces in a real nav robot

Several ROS pieces are required by your ROS robot so I will briefly list some key ones
    http://wiki.ros.org/move_base
    http://wiki.ros.org/slam_gmapping
    The ROS driver for his lidar, in this case it is the Neato XV_lidar (you must research this, to involved for this demo)
    You generally run a ROS slave laptop and on that show RVIZ. The laptop runs same ROS as your robot where your robot is ROS master

These will not run a robot on their own but offer examples of the sort of things that may be required for special launch files specific to ones own robots but in a general way have the main pieces that will in some way have to be supplied for two phases used in a robot that is to use a Lidar to first map a room and then later navigate around the room once the map has been formed.

These examples do not include a costmap so these are simple in that way.  A costmap prevents the robot from hitting things that are placed in front of it since the original map was formed.

First off one has to make a map of the area.   
    - The user saves any old maps perhaps for other areas and cleans out a folder in home dir called ~/.ros/slam
    -The user starts his robot with a ROS launch file for his project.  In this case project is robo_mag
        linux> roslaunch robo_mag droid_mapmaker.launch
    - The user starts slam_gmapping having set it to use /scan topic (standard output for lidar generally)
        linux> rosrun gmapping slam_gmapping scan:=scan
    - The user drives around his robot perhaps looking at RVIZ on a laptop on slave ROS to his robot as Master
    I am not going to explain that just now as it is complex and distracts from my simple mapping explanation
        linux> rosrun teleop_twist_keyboard teleop_twist_keyboard.py           (Then drive around till room seems fully mapped)
    - Save the map (super important step!)   
        linux> rosrun map_server map_saver -f myNewMap-ils                     (saves two files that are the 'map')

Once a map has been made the next step which can be done over and over is to navigate around using this map
    - The user places his map files from mapping into ~/.ros/slam  
    - The user edits his own what I am calling 'maprunner' launch file for the map_saver line to use name of his map
    - The user needs to start his robot with a ROS launch file suitable for his project. In this case my project is robo_mag
        linux> roslaunch robo_mag droid_maprunner.launch
    - The user needs to have a component such as move_base and start it (sorry, not defined in this simple example)
        linux> roslaunch magni_nav move_base.launch
    - The user defines new location(s) for his robot to move to using protocol for move_base
