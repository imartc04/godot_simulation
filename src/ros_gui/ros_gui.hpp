
#pragma once

#include <godot_cpp/classes/control.hpp>

namespace godot
{

    /**
     *Godot node to initialize ROS system wihing Godot 
     *
     */
    class CRosGui : public Control
    {
        GDCLASS(CRosGui, Control)

    protected:
        /**
         * @copydoc godot::CSensorBasicCamera::_bind_methods
         */
        static void _bind_methods();

    public:
        /**
         * @copydoc godot::CSensorBasicCamera::_ready
         */
        void _ready() override;

        void init_ros_system();
    };

} // namespace godot