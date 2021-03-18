#!/usr/bin/python3
import rospy
from kamigami_msgs.msg import KamigamiCommand
from sensor_msgs.msg import Imu
import math

"""
Just try to get the kamigami to have controlled steps
Going to set speed to some reasonable value until we are at a peak in angular velocity.
Then stop.
"""

angular_velocity = [0, 0, 0]

def main():
    pwm = .5;
    rate = rospy.Rate(100)

    rospy.Subscriber('imu/data_raw', Imu, update_vel)
    cmd_pub = rospy.Publisher('kamigami/cmd', KamigamiCommand, queue_size=10)

    msg = KamigamiCommand()
    msg.motor_left = 0
    msg.motor_right = pwm
    cmd_pub.publish(msg)

    while (not rospy.is_shutdown()) or (not step_completed()):
        rate.sleep()

    msg = KamigamiCommand()
    msg.motor_left = 0
    msg.motor_right = 0
    cmd_pub.publish(msg)

def step_completed():
    threshold = .5
    mag = math.sqrt(angular_velocity[0]**2 + angular_velocity[1]**2 + angular_velocity[2]**2)
    print(mag)
    return mag < threshold

def update_vel(imu_data):
    angular_velocity[0] = imu_data.angular_velocity.x
    angular_velocity[1] = imu_data.angular_velocity.y
    angular_velocity[2] = imu_data.angular_velocity.z

if __name__ == '__main__':
    rospy.init_node('kamigami_step', anonymous=True)
    print("Kamigami step started")
    main()
