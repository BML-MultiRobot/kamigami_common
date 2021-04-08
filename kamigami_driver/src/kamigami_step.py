#!/usr/bin/python3
import rospy
from kamigami_msgs.msg import KamigamiCommand
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

"""
Just try to get the kamigami to have controlled steps
Going to set speed to some reasonable value until we are at a peak in angular velocity.
Then stop.
"""

def main():
    pwm = .6
    rate = rospy.Rate(100)
    n_steps = 10
    rospy.on_shutdown(shutdown)
    ang_desired = .1
    threshold = .02

    # Calibrate first
    print("\nCalibrating!")
    calibration_time = 3
    calibration_start = rospy.get_time()
    x_ang_vels = []
    x_tilt = []
    while (rospy.get_time() - calibration_start) < calibration_time:
        x_ang_vels.append(angular_velocity[0])
        x_tilt.append(get_tilt())
        rate.sleep()
    x_ang_bias = sum(x_ang_vels)/len(x_ang_vels)
    x_tilt_start = sum(x_tilt)/len(x_tilt)
    print("\nCalibration finished!")
    print(f"Gyro x bias: {x_ang_bias}")
    print(f"Starting tilt: {x_tilt_start}")

    # Take steps
    for i in range(n_steps):
        last = rospy.get_time()
        ang = 0
        # while (not step_completed()) and (not rospy.is_shutdown()):
        while (abs(ang - ang_desired) > threshold) and (not rospy.is_shutdown()):
            if i % 2 == 0:
                ang_desired = abs(ang_desired)
                send_cmd(pwm, 0)
            else:
                ang_desired = -abs(ang_desired)
                send_cmd(0, pwm)
            cur = rospy.get_time()
            gyro_estimate = ang + (cur - last) * (angular_velocity[0] - x_ang_bias)
            accel_estimate = get_tilt() - x_tilt_start
            last = cur
            # Complementary filter equation
            # TODO: Refine parameters based on cutoff frequencies, sampling times
            ang = .98 * (gyro_estimate) + .02 * accel_estimate
            # Only trust accelerometer if it is reasonable
            # if abs(gyro_estimate - accel_estimate) < .1:
            #     ang = .98 * (gyro_estimate) + .02 * accel_estimate
            # else:
            #     ang = gyro_estimate
            ang_pub.publish(ang)
            print(f"\nTarget Angle: {ang_desired}")
            print(f"Angle: {ang}")
            print(f"Gyro Estimate: {gyro_estimate}")
            print(f"Accel Estimate: {accel_estimate}")
            rate.sleep()

        # Skip out of loop if we are exiting program
        if rospy.is_shutdown():
            break
        print("\nFinished step!")
        send_cmd(0, 0)
        rospy.sleep(.2)

    shutdown()

def step_completed():
    threshold = .055
    # mag = math.sqrt(angular_velocity[0]**2 + angular_velocity[1]**2 + angular_velocity[2]**2)
    # print(f"\nMagnitude: {mag}")
    # print(f"\nX: {abs(angular_velocity[0])}")
    return abs(angular_velocity[0] > threshold)

def send_cmd(motor_left, motor_right):
    msg = KamigamiCommand()
    msg.motor_left = motor_left
    msg.motor_right = motor_right
    cmd_pub.publish(msg)

def shutdown():
    """
    Turn off motors are print a friendly message on shutdown
    """

    send_cmd(0, 0)
    print("Shutting down")

def get_tilt():
    return math.atan2(linear_acceleration[1], math.sqrt(linear_acceleration[1] * linear_acceleration[1] + linear_acceleration[2] * linear_acceleration[2]))

def update_vel(imu_data):
    angular_velocity[0] = imu_data.angular_velocity.x
    angular_velocity[1] = imu_data.angular_velocity.y
    angular_velocity[2] = imu_data.angular_velocity.z
    linear_acceleration[0] = imu_data.linear_acceleration.x
    linear_acceleration[1] = imu_data.linear_acceleration.y
    linear_acceleration[2] = imu_data.linear_acceleration.z


angular_velocity = [0, 0, 0]
linear_acceleration = [0, 0, 0]
rospy.Subscriber('imu/data_raw', Imu, update_vel)
cmd_pub = rospy.Publisher('kamigami/cmd', KamigamiCommand, queue_size=10)
ang_pub = rospy.Publisher('ang', Float64, queue_size=10)

if __name__ == '__main__':
    rospy.init_node('kamigami_step', anonymous=True)
    print("Kamigami step started")
    main()
