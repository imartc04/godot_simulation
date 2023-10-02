
#pragma once

/**
 * @brief Shared library in charge of managing the ROS system.
 */

extern "C" void ros_manager_init_ros_system();

extern "C" bool ros_manager_initialized();
