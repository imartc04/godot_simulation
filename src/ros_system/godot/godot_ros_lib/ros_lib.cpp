#include "ros_lib.hpp"

#include "ros_system/publisher/bridge_ros_pub.hpp"
#include "ros_system/publisher/ros1_pub.hpp"
#include "ros_system/godot/godot_ros_data_parsers.hpp"
#include "ros_system/subscriber/bridge_sim_subs.hpp"
#include "ros_system/subscriber/ros1_subscriber.hpp"
#include "common/pub_if.hpp"

using namespace std;

std::vector<shared_ptr<void>> g_aux_storage;

// Macro to generate API implementation of a publisher
#define DEF_PUB(API_NAME, PARSE_FUNC, ROS_TYPE, SIM_TYPE)                                                                                                   \
    shared_ptr<CPubBufferIf<SIM_TYPE>> API_NAME(CRosSimPubConfig const &f_config)                                                                           \
    {                                                                                                                                                       \
        auto l_bridge = make_shared<CBridgeRosPub<SIM_TYPE, CRos1Publisher<ROS_TYPE>>>();                                                                   \
        CBridgeRosPubConfig<SIM_TYPE, CRos1Publisher<ROS_TYPE>> l_config = {f_config, std::bind(PARSE_FUNC, std::placeholders::_1, std::placeholders::_2)}; \
        l_bridge.config(l_config);                                                                                                                          \
        l_bridge.init();                                                                                                                                    \
        g_aux_storage.push_back(std::static_pointer_cast<void>(l_bridge));                                                                                  \
        return l_bridge.getPubIf();                                                                                                                         \
    }

DEF_PUB(createImagePub, godotImgtoROSImg, ::sensor_msgs::Image, CGodotImgROS)

DEF_PUB(createJointPub, godotJointDataToROSStr, ::std_msgs::String, CJointData)

void createJointMsgSubs(CRosSimSubsConfig const &f_config)
{
    // Create bridge object
    auto l_bridge = make_shared<CBridgeSimSubs<CROS1Subscriber, CJointData>>();

    // Configure bridge
    CBridgeSimSubsConfig<CROS1Subscriber<::std_msgs::String>, CJointData> l_config = {
        {f_config.enabled, f_config.topic_name, f_config.queue_size},
        std::bind(jsonStringToJointData, std::placeholders::_1, std::placeholders::_2),
        f_config.new_data_func};

    l_bridge.set_config(l_config);

    l_bridge.init();
    g_aux_storage.push_back(std::static_pointer_cast<void>(l_bridge));

    return l_bridge.getPubIf();
};
