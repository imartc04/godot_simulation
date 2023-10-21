
#pragma once 

#include "interfaces/pub_buffer_if.hpp"

#include <godot_cpp/classes/image.hpp>
#include "ros_system/publisher/pub_sim_config.hpp"
#include "ros_system/subscriber/subs_config.hpp"
#include "interfaces/joint_data.hpp"

/**
 * Shared library with API to manage ROS objects
 * 
*/
extern "C" CPubBufferIf<::godot::Image> createImagePub(CRosSimPubConfig const & f_config);

extern "C" CPubBufferIf<CJointData> createJointPub(CRosSimPubConfig const & f_config);

extern "C" void createJointMsgSubs(CRosSimSubsConfig const & f_config );






