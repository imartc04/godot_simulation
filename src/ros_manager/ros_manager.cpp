

#include "ros_manager.hpp"

bool g_initialized = false;

void ros_manager_init_ros_system()
{
    g_initialized = true;
}

bool ros_manager_initialized()
{
    return g_initialized;
}