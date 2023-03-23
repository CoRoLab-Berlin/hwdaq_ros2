# digit_msgs

Repo with all Digit ROS2 messages


## Description

The package contains different message types:

1. DigitState
1. DigitCommand
<!-- 1. DigitImu
1. DigitBase -->
1. DigitMotors
1. DigitJoints

**DigitState** is published by the lowlevel api connection node. 

**DigitCommand** contains everthing to control Digit. 

The other msgs are for debuging and plotting. 


## Installation
Depends on msgs:
- std_msgs
- geometry_msgs
- sensor_msgs


Other dependencies
- rosidl_default_generators
- rosidl_interface_packages
- ros2launch
- ament_cmake


## Usage
The msgs contain:

### DigitState

`motor_{xxx}` stands for position, velocity and torque. (e.g. `motor_position`)

`joint_{xxx}` stands for position and velocity. (e.g. `joint_position`)

`{xxx}_limits` stands for torque, velocity and damping. (e.g. `torque_limits`)

| Name | Type |
| ----------- | ----------- |
| `header` | std_msgs/Header  |
| `base` | DigitBase | 
| `imu` | DigitImu | 
| `motor_{xxx}` | float64[20] | 
| `joint_{xxx}` | float64[20] | 
| `battery_charge` | int16 | 


### DigitCommand

`{xxx}_des` stands for damping, velocity and torque. (e.g. `torque_des`)

| Name | Type |
| ----------- | ----------- |
| `header` | std_msgs/Header  |
| `{xxx}_des` | float64[20] | 

<!-- ### DigitBase

| Name | Type |
| ----------- | ----------- |
| `translation` | float64[3]  |
| `orientation` | geometry_msgs/Quaternion | 
| `linear_velocity` | float64[3] | 
| `angular_velocity` | float64[3] |  -->

<!-- ### DigitImu

| Name | Type |
| ----------- | ----------- |
| `orientation` | geometry_msgs/Quaternion | 
| `angular_velocity` | float64[3] | 
| `linear_acceleration` | float64[3] | 
| `magnetic_field` | float64[3]  | -->


### DigitMotors

Contains all motors as names. 
So it is possible to publish all motors separately. But only one value for each motor. 

E.g.

| Name | Type |
| ----------- | ----------- |
| `header` | std_msgs/Header  |
| `left_hip_roll` | float64 | 

### DigitJoints

See **DigitMotors**


## Support
manuel.weiss@bht-berlin.de



## Authors and acknowledgment
Manuel Weiss

## License
MIT


## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.
