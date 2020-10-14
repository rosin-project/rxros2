# RxROS2 Python Examples

This package contains the RxROS2 Python examples.

The purpose of the examples is to:

1. demonstrate how a RxROS2 Python package is setup and build.
1. demonstrate how a RxROS2 Python program is composed.

The RxROS2 Python examples contains four nodes. Two nodes demonstrate how to construct a simple `talker` using either a class or the `create_note(..)` function. The remaining two nodes demonstrate how to create a simple `listener` again using either a class or the `create_note(..)` function.

## Setup and Installation

Please follow the installation instructions on the RxROS2 main page. The examples will be built as part of the main build. After having completed the `colcon` command you can to start the nodes  as described in the next section.

## Starting the RxROS2 Python Nodes

After having built RxROS2, you may try to start any of the following RxROS2 nodes:

```bash
# RxROS2+RxCpp based nodes
ros2 run rxros2_py_examples talker_new_style
ros2 run rxros2_py_examples talker_old_style
```

The `talker` nodes can be combined with any of the `listener` nodes:

```bash
# RxROS2+RxCpp based nodes
ros2 run rxros2_py_examples listener_new_style
ros2 run rxros2_py_examples listener_old_style
```

any combination of `talker` and `listener` should work.
