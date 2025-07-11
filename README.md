# ros2_control_micro

An Arduino LED Blink Program with ROS 2 Control. Here the arduino communicated with ROS 2 through the Serial.

## Usage

Clone this repository

```bash
git clone https://github.com/rahgirrafi/ros2_control_micro.git --recursive 
```

## Dependecies
This package depends on serial-ros2 package from RoverRobotics-forks. You need to first build it.

from the root directory of serial-ros2:

```bash
mkdir build
cd build
cmake ..
make
sudo make install

```
