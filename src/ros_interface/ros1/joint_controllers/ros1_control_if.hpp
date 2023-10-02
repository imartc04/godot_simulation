#pragma once

#include <vector>


namespace godot
{
    /**
     * Interface for ros1 control
     * 
     * This interface has been created to abstract the joint controllers from 
     * the method of controlling them, for example ros1_control, ros2_control or topic based. 
     * This approach also allows easy integration of new types of controlling the jonts.
    */
    class Ros1ControlIf
    {

        public:
            Ros1ControlIf(){};
            virtual ~Ros1ControlIf() = default;

            /** Method where the user must provide new values to command joints velocities
             * 
             * The values of the vector are by default assumed to be angular x,y,z followed by linear x,y,z
            */
            virtual void set_new_joint_reads(std::vector<double> const & ) = 0;

            /** Mehod to read last values of the joints. The vector is assued to be the same of provided in method \ref Ros1ControlIf::set_new_joint_reads*/
            virtual std::vector<double> get_joint_commands() = 0;
    };


}