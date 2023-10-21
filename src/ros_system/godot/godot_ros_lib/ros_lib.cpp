#include "ros_lib.hpp"

#include "ros_system/publisher/bridge_ros_pub.hpp"
#include "ros_system/publisher/ros1_pub.hpp"
#include "ros_system/godot/godot_ros_data_parsers.hpp"


using namespace std;

std::vector<shared_ptr<void>> g_aux_storage;

shared_ptr<CPubBufferIf<CGodotImgROS>> createImagePub(CRosSimPubConfig const & f_config)
{

    //Create bridge object
    auto l_bridge = make_shared<CBridgeRosPub<CGodotImgROS, CRos1Publisher<::sensor_msgs::Image>>>();


    //Configurate bridge
    CBridgeRosPubConfig<CGodotImgROS, CRos1Publisher<::sensor_msgs::Image>> l_config = {f_config, std::bind(godotImgtoROSImg, std::placeholders::_1, std::placeholders::_2) };

    //Initialize bridge
    l_bridge.init();

    //Return buffer pub if
    return l_bridge.getPubIf();

}


extern "C" CPubBufferIf<CJointData> createJointPub(CRosSimPubConfig const & f_config);


extern "C" void createJointMsgSubs(CRosSimSubsConfig const & f_config );




