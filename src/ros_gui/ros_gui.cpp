
#include "ros_gui.hpp"
#include "ros_manager/ros_manager.hpp"
#include "godot_cpp/core/class_db.hpp"

using namespace godot;


void CRosGui::_bind_methods()
{

    //    ADD_PROPERTY(PropertyInfo(Variant::STRING, "ros1_node_name"), "set_ros1_node_name", "get_ros1_node_name");
    ClassDB::bind_method(D_METHOD("init_ros_system"), &CRosGui::init_ros_system);

}

void CRosGui::_ready()
{
    //set_custom_minimum_size(godot::Vector2(100, 30));
    set_anchors_preset(::godot::Control::LayoutPreset::PRESET_FULL_RECT);

}


void CRosGui::init_ros_system()
{
    ros_manager_init_ros_system();
}
