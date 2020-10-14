# RxROS2 C++ Examples

This package contains the RxROS2 C++ examples.

The purpose of the examples are to
1. demonstrate how a RxROS2 C++ project is setup and build.
2. demonstrate how a RxROS2 C++ program is composed.

The RxROS2 C++ examples contains four notes. Two notes demonstrate how to construct a simple `talker` using either a class or the create_note function. The remaining two notes demonstrate how to create a simple `listener` again using either a class or the create_note function.

## Setup and Installation

Please follow the installation instruction of the RxROS2 main page. Part of building the rxros2_ws also includes building the examples. After having completed the `colcon` command it will possible to startup the nodes  as described in the next section.

## Starting the RxROS2 C++ Nodes

After having built RxROS2, you may try to start any of the following RxROS2 nodes:

```bash
# RxROS2+RxCpp based nodes
ros2 run rxros2_cpp_examples talker_new_style
ros2 run rxros2_cpp_examples talker_old_style
```

The talker nodes can be combined with any of the listener nodes:

```bash
# RxROS2+RxCpp based nodes
ros2 run rxros2_cpp_examples listener_new_style
ros2 run rxros2_cpp_examples listener_old_style
```

any combination of `talker` and `listener` should work.
