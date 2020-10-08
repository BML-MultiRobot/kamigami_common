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
- `kamigami/cmd`
##### Published Topics
- `kamigami/imu/data_raw`

## kamigami_bringup

#### Launch Files

##### `joy_teleop.py`
 Startup kamigami to be controlled by some joystick controller.

##### Launched Nodes
 - `kamigami_interface.py`
 - `joy_teleop.py`
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
- `joy`
##### Published Topics
- `kamigami/cmd`

##### `keyboard_teleop.py`
 Takes input from keyboard to control kamigami.
##### Subscribed Topics
- `joy`
##### Published Topics
- `kamigami/cmd`