#pragma once
//Interface class to manage and publish ROS1 topics

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


};

