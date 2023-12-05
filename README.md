# T-14 ROS bridge

ROS bridge to the T-14 car. Useful to perform data collection.

## Requirements
- ROS 2 Humble
- Python
    - python-can
- SocketCAN

## Setup

Note: place your configuration file on the `docs` subdirectory.

### Parameters

- `can_channel`: name of the CAN interface. Default: `can0`
- `can_bustype`: type of CAN interface being used on the host. Only SocketCAN was tested, use at your own risk. Default: `socketcan`
- `read_period`: period in seconds between CAN readings. Default: `0.02`
- `config_path`: path to the configuration file. Check the example provided. Default: `docs/ids.json`

## Subscribers

None

## Publishers

- `ackermann`: Ackermann state (only drive speed and steering angle are populated) (`ackermann_msgs/AckermannDriveStamped`)
- `right_wheels_speed`: Speed measured on the right wheels (`std_msgs/Float32`)
- `left_wheels_speed`: Speed measured on the left wheels (`std_msgs/Float32`)
- `drive_speed`: Measured drive speed (`std_msgs/Float32`)
- `gnd_speed`: Measured ground speed (`std_msgs/Float32`)
