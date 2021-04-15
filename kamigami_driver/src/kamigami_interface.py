#!/usr/bin/python3
import rospy
import math
import board
import busio
from gpiozero import Motor, PWMOutputDevice, DigitalOutputDevice
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
from kamigami_msgs.msg import KamigamiCommand
from sensor_msgs.msg import Imu

class KamigamiInterface():

    def __init__(self, do_calibrate=True):
        """
        Initialize Kamigami interface
        """

        # Setup ROS subscribers, publishers
        pwm_sub = rospy.Subscriber('kamigami/motor_cmd', KamigamiCommand, self.recieve_motor_cmd)
        step_sub = rospy.Subscriber('kamigami/step_cmd', KamigamiCommand, self.self.recieve_step_cmd)
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        
        # Setup action clients, servers
        # None as of now, should be using one for stepping but VREP only supports services/actions for ROS2

        # TODO: Add a debugging mode (multiple levels) or take advantage of ROS's own tools

        # Setup IMU
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = LSM6DS33(i2c)
        # TODO: Read in from config file
        # Running these at slightly above 100 Hz
        # These are currently the default values
        self.sensor.accelerometer_data_rate = Rate.RATE_104_HZ
        self.sensor.gyro_data_rate = Rate.RATE_104_HZ 
        self.sensor.accelerometer_range = AccelRange.RANGE_4G
        self.sensor.gyro_range = GyroRange.RANGE_250_DPS

        # Setup Raspberry Pi Pins
        # TODO: Take pin definitions from a config file
        self.motor_standby = DigitalOutputDevice(17)
        self.motor_right_pwm = PWMOutputDevice(13)
        self.motor_left_pwm = PWMOutputDevice(18)
        self.motor_right_forward = DigitalOutputDevice(27)
        self.motor_right_backward = DigitalOutputDevice(22)
        self.motor_left_forward = DigitalOutputDevice(24)
        self.motor_left_backward = DigitalOutputDevice(23)
        self.pins_on()

        # TODO: Take in constants from some kind of config file
        self._loop_rate = 100
        self._step_pwm = .6
        self._ang_desired = .1
        self._threshold = .02
        self._filter_alpha = .98

        if do_calibrate
            # Calibrate first
            print("\nCalibrating! Please keep robot still.")
            calibration_time = 3
            calibration_start = rospy.get_time()
            x_ang_vels = []
            x_tilt = []
            while (rospy.get_time() - calibration_start) < calibration_time:
                x_ang_vels.append(angular_velocity[0])
                x_tilt.append(get_tilt())
                rate.sleep()
            # Estimate gyro bias in the roll direction by averaging the angular velocity over all samples
            self.x_ang_bias = sum(x_ang_vels)/len(x_ang_vels)
            # Estimate current tilt
            self.x_tilt_start = sum(x_tilt)/len(x_tilt)
            print("\nCalibration finished!")
            print(f"Gyro x bias: {self.x_ang_bias}")
            print(f"Starting tilt: {self.x_tilt_start}")
        
        # Instance variables
        self.roll_estimate = 0

        # Custom shutdown function to stop all motors
        rospy.on_shutdown(self.shutdown)

    def recieve_motor_cmd(self, data):
        """
        Recieve KamigamiCommand message and control motors from that
        """

        self.motor_cmd(data.motor_left, data.motor_right)

    def motor_cmd(self, motor_left_speed, motor_right_speed):
        """
        Set both motors of Kamigami to input values
        """

        # Saturate max/min PWMs
        motor_left_speed = max(min(1, motor_left_speed), -1)
        motor_right_speed = max(min(1, motor_right_speed), -1)

        # Determine pins to set high and output PWM
        if motor_left_speed >= 0:
            self.motor_left_forward.on()
            self.motor_left_backward.off()
            self.motor_left_pwm.value = motor_left_speed
        else:
            self.motor_left_forward.off()
            self.motor_left_backward.on()
            self.motor_left_pwm.value = -motor_left_speed
        if motor_right_speed >= 0:
            self.motor_right_forward.on()
            self.motor_right_backward.off()
            self.motor_right_pwm.value = motor_right_speed
        else:
            self.motor_right_forward.off()
            self.motor_right_backward.on()
            self.motor_right_pwm.value = -motor_right_speed

        # print('Setting motor left to {}'.format(motor_left_speed))
        # print('Setting motor right to {}'.format(motor_right_speed))

    def recieve_step_cmd(self, data):
        self.take_steps(data.left_steps, data.right_steps)

    self take_steps(self, left_steps, right_steps):
        for i in range(n_steps):
        last = rospy.get_time()
        ang = 0
        for _ in range(left_steps):
            while (abs(ang - ang_desired) > threshold) and (not rospy.is_shutdown()):
                ang_desired = abs(ang_desired)
                send_cmd(pwm, 0)
                cur = rospy.get_time()
                gyro_estimate = ang + (cur - last) * (angular_velocity[0] - x_ang_bias)
                accel_estimate = get_tilt() - x_tilt_start
                last = cur
                # Complementary filter equation
                # TODO: Refine parameters based on cutoff frequencies, sampling times
                ang = self._filter_alpha*(gyro_estimate) + (1-self._filter_alpha)*accel_estimate
                # Only trust accelerometer if it is reasonable
                # if abs(gyro_estimate - accel_estimate) < .1:
                #     ang = .98 * (gyro_estimate) + .02 * accel_estimate
                # else:
                #     ang = gyro_estimate
                ang_pub.publish(ang)

        if rospy.is_shutdown():
            break

    def acceleration_to_tilt(self, linear_acceleration):
        return math.atan2(linear_acceleration[1], math.sqrt(linear_acceleration[1] * linear_acceleration[1] + linear_acceleration[2] * linear_acceleration[2])) 

    def update_state(self, last_time):
        """
        Updating state of the Kamigami by
        - polling sensors for new data
        - TODO: filtering
        - publishing new IMU data to imu/data_raw topic
        """

        # Poll IMU sensors
        angular_velocity = self.sensor.gyro
        linear_acceleration = self.sensor.acceleration

        # print("Angular Velocity: {}".format(angular_velocity)
        # print("Linear Acceleration: {}".format(linear_acceleration))

        # Apply drift correction
        angular_velocity[0] = angular_velocity[0] - self.x_ang_bias

        # Update internal roll estimate
        cur_time - rospy.get_time()
        gyro_estimate = self.roll_estimate + angular_velocity[0]*(cur_time-last_time)
        accelerometer_estimate = math.atan2(linear_acceleration[1],
                                 math.sqrt(linear_acceleration[1]*linear_acceleration[1] +
                                           linear_acceleration[2]*linear_acceleration[2]))
        # Simple complementary filter
        # TODO: Use a Kalman filter!
        self.roll_estimate = self._filter_alpha*gyro_estimate + (1-self._filter_alpha)*accelerometer_estiamte
        print (f"Roll: {self.roll_estimate}")

        # Create and populate IMU message
        imu_data = Imu()
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = 'imu_link'
        imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z = angular_velocity
        imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z = linear_acceleration

        # Publish new state data
        self.imu_pub.publish(imu_data)

        return cur_time

    def shutdown(self):
        """
        Disable Kamigami by
        - shutting off motors
        - deactivating pins
        """

        self.motor_cmd(0, 0)
        self.pins_off()
        print("Kamigami interface shutdown.")
    
    def pins_on(self):
        """
        Enable Raspberry Pi pins
        """

        self.motor_standby.on()

    def pins_off(self):
        """
        Deactivate Raspberry Pi pins
        """

        self.motor_standby.off()
        self.motor_right_pwm.off()
        self.motor_left_pwm.off()
        self.motor_right_forward.off()
        self.motor_right_backward.off()
        self.motor_left_forward.off()
        self.motor_left_backward.off()

    def run(self):
        """
        Keep the Kamigami interface running and update state messages
        """

        rate = rospy.Rate(self._loop_rate)
        last_time = rospy.get_time()
        while not rospy.is_shutdown():
            last_time = self.update_state(last_time)
            rate.sleep()
        self.shutdown()
    
if __name__ == '__main__':
    rospy.init_node('kamigami_interface', anonymous=True)
    kamigami = KamigamiInterface()
    print("Kamigami interface started.")
    kamigami.run()
