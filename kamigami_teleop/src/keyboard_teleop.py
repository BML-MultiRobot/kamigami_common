#!/usr/bin/python3
import rospy
import time
from sensor_msgs.msg import Joy
from kamigami_msgs.msg import KamigamiCommand

"""
Note: None of this works ATM
Need to finish modifying to use keyboard instead of joystick
"""

class KeyboardTeleop():

    def __init__(self):
        self.publisher = rospy.Publisher('kamigami/cmd', KamigamiCommand, queue_size=10)
        self.last_input = None

"""
Need to modify callback to be based on keyboard presses instead

    def process_input(self, data):
        axes = data.axes
        input_dict = {'joy_left_y': 1, 'joy_right_y': 4}
        msg = KamigamiCommand()
        msg.motor_left = axes[input_dict['joy_left_y']]
        msg.motor_right = axes[input_dict['joy_right_y']]
        msg.motor_left, msg.motor_right = round(msg.motor_left, 1), round(msg.motor_right, 1)
        if abs(msg.motor_left) <= .1:
            msg.motor_left = 0
        if abs(msg.motor_right) <= .1:
            msg.motor_right = 0
        if msg == self.last_input:
            return
        else:
            self.last_input = msg
            self.publisher.publish(msg)
"""

    def run(self):
        rate = rospy.Rate(100)
        print('Running...')
        while not rospy.is_shutdown():
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        msg = KamigamiCommand()
        msg.motor_left = 0
        msg.motor_right = 0
        self.publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('joy_teleop', anonymous=True)
    controller = JoyTeleop()
    controller.run()
