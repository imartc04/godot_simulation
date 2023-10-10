#pragma once

#include <memory>

// Interface class to manage and publish ROS1 topics

/**
 * Interface class to manage ROS 1 publishers of different types of ROS 1 messages
 *
 */
class CRos1PublisherInterface
{

public:
    /**
     * Initialize ROS 1 publisher
     */
    virtual void init() = 0;

    /**
     * Check if there is some error in ROS 1
     */
    virtual bool ros_error() = 0;

    /**
     * Signal the object to stop publishing
     */
    virtual void stop_publishing() = 0;

    /**
     * Blocking call that publishes ROS 1 messages
     * continuously until ros::ok() fails or some signal to
     * stop the execution is received
     *
     */
    virtual void publish_loop() = 0;

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
