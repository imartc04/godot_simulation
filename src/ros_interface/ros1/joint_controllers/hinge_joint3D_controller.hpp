
#pragma once

#include <godot_cpp/classes/hinge_joint3d.hpp>
#include <memory>
#include "ros_interface/ros1/joint_controllers/ros1_control_if.hpp"
#include "grpc_interface/grpc_subprocess_manager.hpp"
#include "grpc_interface/grpc_godot_helper_macros.hpp"
#include "ros_interface/ros1/joint_controllers/joint_control_manager.hpp"

namespace godot
{

    // /**
    //  * Config struct for CHingeJoint3DController
    //  *
    //  * The config struct holds ros1 control interface, which controls the velocity
    //  * of the hinge joint. A hinge joint in godot is a joint that allows rotation on
    //  * its z-axis controlled by velocity.
    //  *
    //  */
    // struct CHingeJoint3DControllerConfig
    // {
    //     std::unique_ptr<Ros1ControlIf> ros1_control_if;

    // };

    /**
     * Controller for a godot hinge joint
     *
     * The purpose of this class is to use it as a node that acts as a godot joint node
     * between 2 rigid bodie.
     *
     * The config struct holds ros1 control interface, which
     * abstract the joint controller from the medthod of controlling it (ros1_control, ros2_control, topic based, etc.).
     * A hinge joint in godot is a joint that allows rotation on its z-axis controlled by velocity.
     *
     * Use of the class:
     *
     *  - Create a godot node of type CHingeJoint3DController
     *  - Set node configuration through the UI editor or by code with set_config()
     *
     * @see Ros1ControlIf
     */
    class CHingeJoint3DController : public HingeJoint3D
    {
        GDCLASS(CHingeJoint3DController, HingeJoint3D)

    public:
        // Constructor
        CHingeJoint3DController();

        // Destructor
        ~CHingeJoint3DController();

        /**
         * \copydoc CSensorBasicCamera::_init
         */
        void _init();

        /**
         * \copydoc CSensorBasicCamera::_ready
         */
        void _ready() override;

        // Godot process
        // void _process(float delta);

        /**
         * \copydoc CGeneric6DOFController::_physics_process
         */
        void _physics_process(float delta);

        GRPC_DECLARE_METHODS

        void set_control_type(::godot::String f_control_type);

        // get_control_type method
        ::godot::String get_control_type();

        void set_joint_name(::godot::String f_joint_name);

        ::godot::String get_joint_name();

        /**
         * Set controls signal update period
         */
        void set_update_period_ms(float f_update_period_ms);

        /**
         * Get controls signal update period
         */
        float get_update_period_ms();

    protected:
        /**
         * \copydoc CSensorBasicCamera::_bind_methods
         */
        static void _bind_methods();

    private:
        // Joint name
        ::godot::String m_joint_name;

        std::map<::godot::String, EControlTye> m_str_to_control_type_map = {
            {"topic_based", EControlTye::TOPIC_BASED},
            {"ros_control_package", EControlTye::ROS_CONTROL_PACKAGE}};

        std::map<EControlTye, ::godot::String> m_control_type_to_str_map;

        /**
         * Initialization flag
         */
        bool m_initialized = false;

        /**
         * ROS 1 joint control interface
         */
        std::unique_ptr<CJointControlManager> m_control_if;

        // Configuration for child process when used
        CJointControlManagerConfig m_config;

        t_joint_values m_joint_reads;
    };
}