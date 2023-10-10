
#include "hinge_joint3D_controller.hpp"
#include "ros_manager/ros_manager.hpp"

#include "grpc_interface/joint_vel_server_impl.hpp"
#include "utils/utils.hpp"

using namespace godot;
using namespace std;

CHingeJoint3DController::CHingeJoint3DController()
{

    m_control_type_to_str_map = reverse_map(m_str_to_control_type_map);
};

CHingeJoint3DController::~CHingeJoint3DController(){};

GRPC_DEFINE_METHODS(CHingeJoint3DController, m_subprocess_config)

void CHingeJoint3DController::_bind_methods()
{

    GRPC_BIND_GODOT_METHDOS(CHingeJoint3DController)

    ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "control_type"), "set_control_type", "get_control_type");

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

// Godot ready
// void _ready()
// {

// }

void CHingeJoint3DController::_physics_process(float delta)
{

    if (ros_manager_initialized())
    {

        if(!m_initialized)
        {
            // Set initialized
            m_initialized = true;

            // Initialize control
            m_control_if = make_unique<JointVelServerImpl>();

            // Set joint name an throw execption if joint name is not set
            if (m_joint_name != "")
            {
                m_config.control_var_names.push_back(m_joint_name);
            }
            else
            {
                throw std::runtime_error("Joint name not set");
            }

            // Set config
            m_config.grpc_config.enable = true;
            m_config.grpc_config.exec_name = "ros1_pub_node";
            m_config.grpc_config.run_in_new_thread = true;

            m_control_if->config(m_config);

            // Initialize child process
            m_control_if->init();
        }

        // Obtain joint velocity value
        m_joint_reads.set_has_angular_z = true;
        m_joint_reads.set_angular_z(this->get_param(HingeJoint3D::PARAM_MOTOR_TARGET_VELOCITY));

        m_control_if->set_new_joint_reads(m_joint_reads);

        // Set joint velocity with command
        this->set_param(HingeJoint3D::PARAM_MOTOR_TARGET_VELOCITY, m_control_if->get_joint_commands().angular_z());

    }
}
