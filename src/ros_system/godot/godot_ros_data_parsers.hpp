
#pragma once 

#include <cstdint>
#include <godot_cpp/classes/image.hpp>
#include "ros/Image.h"
#include "interfaces/godot_img_ros.hpp"
#include "parser/joint_parser.hpp"
#include "std_msgs/String.h"


/**
 * Functions to parse Godot sensor data types to ROS ones
*/
void godotImgtoROSImg(CGodotImgROS &f_in, ::sensor_msgs::Image &f_out);


/**
 * Parse from ros string msg to CJointData
*/
void godotJointDataToROSStr(CJointData& f_in, ::standar_msgs::String& f_out);


/**
 * Parse from CJointData to ros string
*/
void godotROSStrToJointData(::standar_msgs::String& f_in, CJointData& f_out);