# kamigami_common
ROS packages for Kamigami usage

| Package Name | Description |
| ---- | ---- |
| [kamigami_driver](#kamigami_driver) | Interact with hardware on kamigami iteself |
| [kamigami_bringup](#kamigami_bringup) | Startup kamigami in desired configuration |
| [kamigami_msgs](#kamigami_msgs) | Custom message definitions and message dependencies for other packages |
| [kamigami_sim](#kamgigami_sim) | Interfacing with the kamigami in VREP simulation |
| [kamigami_teleop](#kamigami_teleop) | Manually control the kamigami via remote operation |

## kamigami_driver

### Nodes
##### `kamigami_interface.py`
 Takes velocity commands and sets appropriate motor velocities. Publishes sensor data.
##### Subscribed Topics
- `kamigami/cmd (kamigami_common/KamigamiCommand)` Kamigami velocity command to control motors.
##### Published Topics
- `kamigami/imu/data_raw (sensor_msgs/Imu)` Raw data from IMU on kamigami.

## kamigami_bringup

#### Launch Files

##### `joy_teleop.launch`
 Startup kamigami to be controlled by some joystick controller.

##### Launched Nodes
 - [`kamigami_interface.py`](#kamigami_teleop)
 - [`joy_teleop.py`](#joy_teleoppy)
##### Parameters

## kamigami_msgs

#### Messages

##### `KamigamiCommand.msg`
Velocity command for kamigami.

## kamgigami_sim

#### Nodes

## kamigami_teleop

#### Nodes

##### `joy_teleop.py`
 Takes input from joystick controller to control kamigami.
##### Subscribed Topics
- `joy (sensor_msgs/Joy)` Joystick controller output.
##### Published Topics
- `kamigami/cmd (kamigami_common/KamigamiCommand)` Kamigami velocity command to control motors.

##### `keyboard_teleop.py`
 Takes input from keyboard to control kamigami.
##### Published Topics
- `kamigami/cmd (kamigami_common/KamigamiCommand)` Kamigami velocity command to control motors.