#pragma once

#include <memory>

/**
 * Auxliar class to store dependencies to shared pointer when objects
 * are needed to be manatined in memory
 */
class CAuxDependStorage
{
private:
    /* data */
public:
    CAuxDependStorage(/* args */){};
    ~CAuxDependStorage(){};

    /**
     * Add object dependency to avoid destruction of the object
     * Used for interaction between object callbacks due isolation of
     * ROS in shared libraries
     */
    void add_obj_dep(std::shared_ptr<void> f_obj_dep);
    {
        m_obj_dep.push_back(f_obj_dep);
    }

private:
    std::vector<std::shared_ptr<void>> m_obj_dep;
};
