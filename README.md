# RxROS2

Reactive programming for ROS 2.

## Overview

This repository contains the ROS 2 version of [rxros](https://github.com/rosin-project/rxros).

As the implementation of this new version is rather different from the ROS 1 version, the packages are hosted in a separate repository instead of as a branch in `rosin-project/rxros`.


## Design and use

For more information on the design and use of `rxros2`, please see the `README.md` files in the `rxros2_cpp` and the `rxros2_py` package directories.


## Acknowledgements

<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" alt="rosin_logo" height="60">
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement no. 732287.


## Dependencies

The RxROS2 library depends on and uses the following software:

 1. Ubuntu Focal Fossa 20.04 and ROS2 Foxy Fitzroy
 1. Reactive C++ (RxCpp) v4.0
 1. Reactive Python (RxPy) v3.1.0

Please follow the installation instruction of Ubuntu at
https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview

and the installation instructions of ROS2 at
https://index.ros.org/doc/ros2/Installation/.

Reactive C++ (RxCpp) and reactive Python (RxPy) will be installed as part of installing RxROS2 (see instructions below).
They do not have to be installed separately.

## Setup and Installation

The current installation instructions are manual, but will soon be replaced by proper packages that will allow easy installation of RxROS2. RxROS2 is originally developed and tested of the Eloquent Elusor platform, but we have not been able to provide proper installation instructions for this platform (see issue #2 for a discussion of the problem). Foxy Fitzroy has therefore become the preferred platform for RxROS2 although we have observed a serious issue with rclpy/RxROS2 for this platform (see issue #9). 

Installation of rosdep (depenency package):
```bash
sudo apt install python3-rosdep
sudo rosdep init
```

Installation of colcon (dependency package):
```bash
sudo apt install python3-colcon-common-extensions
```

Installation of RxROS2 for Python and C++:
```bash
mkdir -p $HOME/rxros2_ws/src
cd $HOME/rxros2_ws/src
git clone https://github.com/rosin-project/rxros2.git
cd $HOME/rxros2_ws
rosdep update && sudo apt update
rosdep install --from-paths $HOME/rxros2_ws --ignore-src --rosdistro=foxy
colcon build
```

## Example Package

After having installed RxROS2 try to execute the following RxROS nodes

ros2 run rxros_cpp_examples talker_new_style<br>
ros2 run rxros_cpp_examples talker_old_style<br>
ros2 run rxros_py_examples talker_new_style<br>
ros2 run rxros_py_examples talker_old_style<br>

The talker nodes can be combined with any of the listener nodes

ros2 run rxros_cpp_examples listener_new_style<br>
ros2 run rxros_cpp_examples listener_old_style<br>
ros2 run rxros_py_examples listener_new_style<br>
ros2 run rxros_py_examples listener_old_style<br>

## Creating a ROS2 Package

TBD
