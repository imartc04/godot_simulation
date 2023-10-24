#pragma once


#include <string>


struct CScalarValue
{
    bool valid = false;

    float value = 0.f;
};

/**
 * Interface used to represent unique robot joint information between different systems
*/
struct CJointData
{

    std::string name;

    //Linear velocities
    CScalarValue lin_vel_x;
    CScalarValue lin_vel_y;
    CScalarValue lin_vel_z;

    //Angular velocities
    CScalarValue angular_x;
    CScalarValue angular_y;
    CScalarValue angular_z;
    
    
};
