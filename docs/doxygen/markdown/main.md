
# Introduction

The purpose of this software package is to merge ROS ecosystem with Godot to use the last one as a simulator for robotics.

The sofware divides in the following modules : 

- Virtual sensor library for godot 
- Joint controller library to command robots
- URDF godot importer to generate robot as a Godot scene
  
  
[Github repository](https://github.com/imartc04/godot_simulation)

# Requirements

- Scons (tested 4.5.2)
- Linux (tested Ubuntu 20.04)
- gcc (tested 9.4.0) 
- VScode for eaiser development wih debug and custom configured tasks
- Godot game engine 
- ROS (version depending of what nodes are used)

# Topics

 - [How does Godot connect to ros?](connection_to_ros.md)
 - [Virtual sensors](virtual_sensors.md)
 - [Joint controllers]()


# Supported Godot versions

- 4.1



# TODO Features

- URDF file importer 
- ROS Noetic publisher implementation
- ROS 2 publisher implementation
- ros_control support for joint controllers
- ros2_control support for joint controllers
- ROS Noetic topic based joint control
- ROS 2 topic based joint control
- Support for compilation in debug and release on scons configuration file
- LLVM toolchain support (Clang, lld, lldb, libc++)
- Possible change to Meson build system
