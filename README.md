# ENPM808X - Week 9 Assignment : ROS2 Package for simple Publisher / Subscriber

## Overview and Description

An example of Publisher/Subscriber package for ROS2 written in C++

## License

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Author

Anukriti Singh - MEng Robotics

## Assumptions and Dependencies
ROS2 Humble package is created and tested on ubuntu 20.02 (Linux).
The colcon build is used for building the package. To run, build and source ROS2 Humble


## To build the package

In new tutorial:
```
mkdir ros2_ws/src
cd ros2_ws/src
git clone <repo>
cd ..
source <path to ros2 setup>/install/setup.bash    
colcon build
source install/setup.bash
```
For publisher in new terminal:
```
cd ros2_ws
source <path to ros2 setup>/install/setup.bash    
source install/setup.bash
ros2 run cpp_pubsub talker
```
Parallely open new terminal:
```
cd ros2_ws
source <path to ros2 setup>/install/setup.bash    
source install/setup.bash
ros2 run cpp_pubsub listener
```

## To run Cpplint
```
cd ros2_ws/src
run_cpplint.sh
```

## To run Cppcheck
```
cd ros2_ws/src/
run_cppcheck.sh
```
