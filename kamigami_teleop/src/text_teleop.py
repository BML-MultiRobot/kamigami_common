#!/usr/bin/python3
import rospy
import time
from sensor_msgs.msg import Joy
from kamigami_msgs.msg import KamigamiCommand

class TextTeleop():

    def __init__(self):
        self.publisher = rospy.Publisher('kamigami/cmd', KamigamiCommand, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def run(self):
        print("Running...")
        while not rospy.is_shutdown():
            try:
                left_pwm = float(input("Left PWM: "))
                right_pwm = float(input("Right PWM: "))
                msg = KamigamiCommand(left_pwm, right_pwm)
                self.publisher.publish(msg)
            except:
                print("Unsupported Input")
                break
        self.shutdown()

    def shutdown(self):
        msg = KamigamiCommand()
        msg.motor_left = 0
        msg.motor_right = 0
        self.publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('joy_teleop', anonymous=True)
    controller = TextTeleop()
    controller.run()
