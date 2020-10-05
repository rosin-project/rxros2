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

The current installation instructions are manual, but will soon be replaced by proper packages that will allow easy installation of RxROS2.
RxROS2 is originally developed and tested of the Eloquent Elusor platform, but we have not been able to provide proper installation instructions for this platform (see [issue 2](https://github.com/rosin-project/rxros2/issues/2) for a discussion of the problem).
Foxy Fitzroy has therefore become the preferred platform for RxROS2 although we have observed a serious issue with `rclpy` / RxROS2 for this platform (see [issue 9](https://github.com/rosin-project/rxros9/issues/2)).

Installation of `rosdep` (this will resolve our dependencies):

```bash
sudo apt install python3-rosdep
sudo rosdep init
```

Installation of `colcon` (build tool):

```bash
sudo apt install python3-colcon-common-extensions
```

Build of RxROS2 for Python and C++:

```bash
# create a workspace
# if you already have a workspace, skip this step
mkdir -p $HOME/rxros2_ws/src
cd $HOME/rxros2_ws

# download the repository
git clone https://github.com/rosin-project/rxros2.git src/rxros2

# make sure all dependencies are installed
rosdep update && sudo apt update
rosdep install --from-paths $HOME/rxros2_ws --ignore-src --rosdistro=foxy

# check the output of 'rosdep install ..' and make sure there were no errors

# build the workspace
colcon build
```

If this was successful, do not forget to activate the workspace by executing `source $HOME/rxros2_ws/install/setup.bash` (if using Bash as the shell).


## Examples

After having built RxROS2, you may try to execute any of the following RxROS2 nodes:

```bash
# RxROS2+RxCpp based nodes
ros2 run rxros_cpp_examples talker_new_style
ros2 run rxros_cpp_examples talker_old_style

# RxROS2+RxPy based nodes
ros2 run rxros_py_examples talker_new_style
ros2 run rxros_py_examples talker_old_style
```

The talker nodes can be combined with any of the listener nodes:

```bash
# RxROS2+RxCpp based nodes
ros2 run rxros_cpp_examples listener_new_style
ros2 run rxros_cpp_examples listener_old_style

# RxROS2+RxPy based nodes
ros2 run rxros_py_examples listener_new_style
ros2 run rxros_py_examples listener_old_style
```

any combination of `talker` and `listener` should work.


## Creating an RxROS2 Package

We will describe the steps to create a ROS 2 package based on RxROS2 at a later time.

In essence these packages are regular ROS 2 packages, they simply depend on `rxros2_cpp` or `rxros2_py`.
