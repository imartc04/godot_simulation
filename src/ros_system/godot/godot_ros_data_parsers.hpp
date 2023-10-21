
#pragma once 

#include <cstdint>
#include <godot_cpp/classes/image.hpp>
#include "ros/Image.h"
#include "interfaces/godot_img_ros.hpp"

/**
 * Functions to parse Godot sensor data types to ROS ones
*/
void godotImgtoROSImg(CGodotImgROS &f_in, ::sensor_msgs::Image &f_out);

