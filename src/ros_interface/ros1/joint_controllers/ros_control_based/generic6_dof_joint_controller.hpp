
#pragma once

#include <godot_cpp/classes/generic6_dof_joint3d.hpp>
#include <memory>
#include "ros1_control_if.hpp"

namespace godot
{

    /**
     * Config struct for CGeneric6DOFController
     *
     * The config struct holds a array of 6 ros1 control interfaces, which control
     * the 6 DOF of angular an linear velocities of the joint.
     *
     */
    struct CGeneric6DOFControllerConfig
    {
        std::array<std::unique_ptr<Ros1ControlIf>, 6u> ros1_control_if;
    };

    /**
     * Controller for a godot generic 6 DOF joint
     *
     * The purpose of this class is to use it as a node that acts as a godot joint node
     * between 2 rigid bodies. 
     *
     * The configuration member holds a array of 6 ros1 control interfaces, which
     * abstract the joint controller from the medthod of controlling it (ros1_control, ros2_control, topic based, etc.). As with all joints in godot
     * the joint can only be controlled by setting the target velocity.
     * In particular, this joint can set linear and angular velocities in x,y,z directions.
     *
     * Use of the class:
     *
     *  - Create a godot node of type CGeneric6DOFController
     *  - Set node configuration through the UI editor or by code with set_config()
     *
     * @see Ros1ControlIf
     *
     */
    class CGeneric6DOFController : public Generic6DOFJoint3D
    {
        GDCLASS(CGeneric6DOFController, Generic6DOFJoint3D)

    public:
        // Constructor
        CGeneric6DOFController();

        // Destructor
        ~CGeneric6DOFController();

        // Set config
        void set_config(CGeneric6DOFControllerConfig &&f_config);

        /**
         *
         * \copydoc CSensorBasicCamera::_init
         */
        void _init();

        /**
         *
         * \copydoc CSensorBasicCamera::_ready
         */
        void _ready();

        // Godot process
        // void _process(float delta);

        /**
         * Godot method called for this node to process it each physics frame
         * The physics frame rate is set in the godot project settings and is 60 by default
         * Although the physics frame rate is highly stable the delta parameter is passed
         * so that the developer can account for any small variations on it
         *
         * @param delta The time in seconds since the last physics frame
         */
        void _physics_process(float delta);

    protected:
        /**
         * \copydoc CSensorBasicCamera::_bind_methods
         */
        static void _bind_methods();

    private:
        /**
        6 controllers
        The first three for x,y,z angular velocity
        The last three for x,y,z linear velocity
        */
        CGeneric6DOFControllerConfig m_config;
    };

}