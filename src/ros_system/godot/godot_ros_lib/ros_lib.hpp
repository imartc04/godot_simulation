
#pragma once 

#include "interfaces/pub_buffer_if.hpp"

#include <godot_cpp/classes/image.hpp>
#include "ros_system/publisher/pub_sim_config.hpp"
#include "ros_system/subscriber/subs_config.hpp"
#include "common/joint_data.hpp"


/**
 * Shared library with API to manage ROS objects
 * 
*/
extern "C" CPubBufferIf<::godot::Image> roslib_createImagePub(CRosSimPubConfig const & f_config);

extern "C" CPubBufferIf<CJointData> roslib_createJointPub(CRosSimPubConfig const & f_config);

extern "C" void roslib_createJointMsgSubs(CROSSubscribConfig<CJointData> const & f_config );






