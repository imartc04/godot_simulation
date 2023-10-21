#pragma once

#include "aux_depend_storage.hpp"


class CROS1SubscriberIf : public CAuxDependStorage
{
public:

    virtual void init() = 0;

    //virtual bool ros_error() = 0;

    virtual void stop() = 0;


};

