
#pragma once

#include <godot_cpp/classes/hinge_joint3d.hpp>
#include <memory>
#include "ros1_control_if.hpp"

namespace godot
{

    /**
     * Config struct for CHingeJoint3DController
     *
     * The config struct holds ros1 control interface, which controls the velocity
     * of the hinge joint. A hinge joint in godot is a joint that allows rotation on
     * its z-axis controlled by velocity.
     *
     */
    struct CHingeJoint3DControllerConfig
    {
        std::unique_ptr<Ros1ControlIf> ros1_control_if;
    };

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

        // Set config
        void set_config(CHingeJoint3DControllerConfig &&f_config);

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

    protected:
        /**
         * \copydoc CSensorBasicCamera::_bind_methods
         */
        static void _bind_methods();

    private:
        CHingeJoint3DControllerConfig m_config;
    };

}