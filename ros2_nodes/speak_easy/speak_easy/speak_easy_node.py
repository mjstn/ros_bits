# Ros2 python node to listen for requests to speak a message.
#
# For each message received on topic speak_command  voice a wave file
#
# Dependancies (besides what you see for python imports just below)
# This script will use the alsa tools of  'aplay' and 'amixer' using apt install alsa-tools
# This defines the sound card device seen with 'aplay -l' 
# You can create a /etc/asound.conf and set soundCardOption and soundCard both to '' if desired
#
# Test this sending commands:  ros2 topic pub /speak_command std_msgs/String "data: car_starting"
#
# Run node from colcon_ws:  python3 src/speak_easy/speak_easy/speak_easy_node.py
#
# Script put together by Mark-Toys.com (mjstn2011@gmail.com)
#
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# We are hard coding some configuration settings but it would be 'nice' to read params
volume='99%'                # The volume we will set the device to play when using 'aplay'

# You can also set Option and Card  to '' when you set  default is not set in /etc/asound.conf
soundCardOption='-D '       # sound card device seen with 'aplay -l'  
soundCard='plughw:CARD=UACDemoV10,DEV=0'     # 'Honk' Usb speaker device seen with 'aplay -l' for your usb speaker

# The soundfiles must reside in this folder with an absolute path
soundFileFolder='/home/ubuntu/SoundFiles/'

# ------------------------------------------------------------------------------
# This node receives these text commands and will map them to specific WAV files
# The mapping to commands from the topic to wav files is below        
# This allows easy reconfigureation for command strings to sounds as below
# THIS is the place to modify for customizations as well as soundFileFolder perhaps
cmd_to_file_map = [
    ['alarm1', 'AlertSiren.wav'],
    ['alarmAlien', 'AlarmForAlien.wav'],
    ['redAlert_1', 'RedAlert3cycles.wav'],
    ['police_siren_1', 'PoliceSiren.wav'],
    ['police_siren_1', 'PoliceSiren.wav'],
    ['police_siren_2', 'PoliceSiren2.wav'],
    ['fire_siren_1', 'FireTruckSiren_.wav'],
    ['fire_siren_2', 'FireEngineWithHonk.wav'],
    ['car_starting', 'CarStarting.wav'],
    ['car_horn_1', 'CarHorn2shortBeeps.wav'],
    ['car_horn_2', 'CarHornDouble.wav'],
    ['car_horn_3', 'CarHornAohhga.wav'],
    ['glas_breaking_1', 'GlassBreaking1.wav'],
    ['glas_breaking_2', 'GlassBreaking2.wav'],
    ['glas_breaking_3', 'GlassBreaking3.wav'],
    ['glas_breaking_4', 'GlassWindowBreakImpact.wav'],
    ['robot_chatter_1', 'RobotChatter1.wav'],
    ['robot_chatter_2', 'RobotChatter2.wav'],
    ['robot_chatter_3', 'RobotChatter3.wav'],
    ['gunShot1', 'GunShot1.wav'],
    ['gunShotSilencer', 'GunShotSilencer.wav'],
    ['shotgun_1', 'Shotgun1.wav'],
    ['shotgun_2', 'ShotgunAndReload.wav'],
    ['explosion_1', 'Explosion1.wav'],
    ['explosion_2', 'Explosion2p5sec.wav'],
    ]
# ------------------------------------------------------------------------------

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            String,
            'speak_command',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    # Every command received on inbound subscribed topic gets to this point.
    # We will see if the command matches a known command and play the soundfile if recognized
    def listener_callback(self, msg):
        self.get_logger().info('Command: "%s"' % msg.data)
        for cmd,soundFile in cmd_to_file_map:
            if msg.data == cmd:
                uxcmd = 'aplay ' + soundCardOption + soundCard + ' ' + soundFileFolder + soundFile 
                print('Use command: ',uxcmd)
                os.system(uxcmd)


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

