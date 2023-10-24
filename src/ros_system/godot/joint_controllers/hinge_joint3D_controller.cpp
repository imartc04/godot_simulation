
#include "hinge_joint3D_controller.hpp"
#include "ros_manager/ros_manager.hpp"
#include "utils/utils.hpp"
#include "ros_system/godot/godot_ros_lib/ros_lib.hpp"

using namespace godot;
using namespace std;

CHingeJoint3DController::CHingeJoint3DController(){

};

CHingeJoint3DController::~CHingeJoint3DController(){};


void CHingeJoint3DController::_bind_methods()
{

   //TODO : Add properties for ROS, ROS2  and DDS publishers and subscribers 

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "joint_name"), "set_joint_name", "get_joint_name");

    ADD_PROPERTY(PropertyInfo(Variant::REAL, "update_period_ms"), "set_update_period_ms", "get_update_period_ms");
}

// set_update_period_ms method
void CHingeJoint3DController::set_update_period_ms(float f_update_period_ms)
{
    m_config.update_period_ms = f_update_period_ms;
}

// get_update_period_ms method
float CHingeJoint3DController::get_update_period_ms()
{
    return m_config.update_period_ms;
}

// set_control_type method
void CHingeJoint3DController::set_control_type(::godot::String f_control_type)
{
    m_control_type = m_str_to_control_type_map[static_cast<String>(f_control_type)];
}

// get_control_type method
String CHingeJoint3DController::get_control_type()
{
    return m_control_type_to_str_map[m_control_type];
}

// set_joint_name method
void CHingeJoint3DController::set_joint_name(::godot::String f_joint_name)
{
    m_joint_name = f_joint_name;
}

// get_joint_name method
String CHingeJoint3DController::get_joint_name()
{
    return m_joint_name;
}

// Godot init
void CHingeJoint3DController::_init()
{
}


void CHingeJoint3DController::_ready()
{
}


void CHingeJoint3DController::_physics_process(float delta)
{

    if (ros_manager_initialized())
    {

        if (!m_initialized)
        {
            // Set initialized
            m_initialized = true;

            // Set config
            m_controller_if.set_config(m_control_if_config);
            m_controller_if.init();
        }

        auto l_command = m_controller_if.get_last_joint_commands();

        // Set joint velocity with command
        if (l_command.angular_x.valid)
        {
            this->set_param(HingeJoint3D::PARAM_MOTOR_TARGET_VELOCITY, l_command.angular_x.value);
        }
        else
        {
            cout << "Error setting velocity command into hing joint 3D, no valid angular x value" << endl;
        }
        
        CJointData l_data;
        l_data.angular_x.valid = true;
        l_data.angular_x.value = this->get_param(HingeJoint3D::PARAM_MOTOR_TARGET_VELOCITY);

    }
}
