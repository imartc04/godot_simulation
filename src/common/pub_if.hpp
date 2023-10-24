#pragma once

#include <memory>
#include "common/pub_buffer_if.hpp"


template<typename t_sim_data>
class CPubIf
{

public:

    virtual void init() = 0;

    virtual shared_ptr< CPubBufferIf<t_sim_data>> getBuffIf() = 0; 


};
