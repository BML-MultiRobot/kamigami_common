#!/usr/bin/python3
import rospy
import time
import sys
from sensor_msgs.msg import Joy
from kamigami_msgs.msg import KamigamiCommand

class CommandTeleop():

    def __init__(self, left_pwm, right_pwm):
        self.publisher = rospy.Publisher('kamigami/cmd', KamigamiCommand, queue_size=10)
        self.left_pwm = left_pwm
        self.right_pwm = right_pwm
        rospy.on_shutdown(self.shutdown)

    def run(self):
        msg = KamigamiCommand(self.left_pwm, self.right_pwm)
        zero_msg = KamigamiCommand(0, 0)
        rate = rospy.Rate(1)
        print("Running...")
        run_period = .3
        rest_period = .8
        while not rospy.is_shutdown():
            try:
                self.publisher.publish(msg)
                rospy.sleep(run_period)
                self.publisher.publish(zero_msg)
                rospy.sleep(rest_period)
            except:
                print("Oh no!")
                break
            #rate.sleep()
        self.shutdown()

    def shutdown(self):
        msg = KamigamiCommand()
        msg.motor_left = 0
        msg.motor_right = 0
        self.publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('command_teleop', anonymous=True)
    left_pwm = float(sys.argv[1])
    right_pwm = float(sys.argv[2])
    controller = CommandTeleop(left_pwm, right_pwm)
    msg = KamigamiCommand(left_pwm, right_pwm)
    controller.run()
