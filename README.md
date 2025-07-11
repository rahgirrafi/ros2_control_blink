# ros2_control_micro

An Arduino LED Blink Program with ROS 2 Control. Here the arduino communicated with ROS 2 through the Serial.


The read and write method in arduino_hardware.cpp is used for sending data to Arduino. The arduino need to be programmed so that it reads serial data whenever it is available.

Interested users who are experience with arduino should be able to customize the code to make it usable for their fucntion.


As micro_ros cannot run on low resource devices like Arduino Uno or nano, serial communication is a very simple yet effective way for them to communicate with ROS 2 interface.
