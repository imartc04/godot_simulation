#include "ros1_pub.hpp"
#include <boost/process.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <map>
#include <variant>
#include <string>
#include <memory>
#include <exception>

using namespace boost::process;
using namespace std;
using namespace godot;

string g_ros_msg_image = "sensor_msgs/Image";

boost::interprocess::named_mutex *g_segment_data_mutex = nullptr;

boost::interprocess::managed_shared_memory *g_segment_data = nullptr;

std::string g_shm_data_name = "";

std::map<std::string, uint64_t> g_mesage_size_map = {{g_ros_msg_image, sizeof(::sensor_msgs::Image)}};

// Help method on how to use appliacation
void help()
{
    std::cout << "Usage: ros1_pub <ros1_msg_type> <shm_name> <config_shm_name> <mutex_name>" << std::endl;
    std::cout << "Example: ros1_pub sensor_msgs/Image image_data image_config image_mutex" << std::endl;
}

// Callback function to generate ros1 message from shared memory passed data
template <typename t_message>
t_message gen_ros1_msg_from_shm_data()
{
    // Create ros1 message object
    t_message ros1_msg;

    // lock mutex using lock_guard
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*g_segment_data_mutex);

    auto shm_data = g_segment_data->find<t_message>(g_shm_data_name.c_str()).first;

    // Check if data is valid
    if (shm_data == nullptr)
    {
        std::cout << "Error: ros1 message data not found" << std::endl;

        throw std::runtime_error("Error: ros1 message data not found");
    }

    // Copy data from shared memory to ros1 message
    ros1_msg = *shm_data;

    return std::move(ros1_msg);
}

/**
 * Application that creates a ros1 publisher for a given ros1 message type
 *
 * The application is intented to work with interprocess shared memory communication
 * to share the next para meters listed by the order of the arguments:
 *
 *  - ros1_msg_type: Type of ros1 message to publish
 *  - shm_name: Name of the shared memory segment that will contain the ros1 message
 *  - topic_type : String representing the type of the topic to publish
 *  - config_shm_name: Name of the shared memory segment that will contain the config data
 *  - mutex_name: Name of the mutex to lock the shared memory segment
 *  - ros_node_name: Name of the ros node
 *
 *
 * Return values :
 *
 *  - 0: Success
 *  - 1: General config error
 *  - 2: Error in ros1 publisher, relaunch node
 *
 */
int main(int argc, char **argv)
{

    // Check passed arguments are correct and print help when needed
    if (argc != 5)
    {
        help();
        return 1;
    }

    std::string ros1_msg_type = argv[1];

    // Get shared memory segment name as second arg
    g_shm_data_name = argv[2];

    // Get shared memory name for the config data
    std::string config_shm_name = argv[3];

    // Get shared memory mutex name
    std::string mutex_name = argv[4];

    // Get ros node name
    std::string ros_node_name = argv[5];

    // Initialize ROS
    if (!ros::isInitialized())
    {
        int l_argc = 0;
        char **l_argv = NULL;
        ros::init(l_argc, l_argv, ros_node_name.c_str());
    }

    // Create mutex object
    boost::interprocess::named_mutex mutex(boost::interprocess::open_or_create, mutex_name.c_str());

    g_segment_data_mutex = &mutex;

    // uint64_t shm_size = g_mesage_size_map[ros1_msg_type];

    // Remove shared memory on construction and destruction
    // struct shm_remove
    // {
    //     shm_remove() { shared_memory_object::remove(shm_name); }
    //     ~shm_remove() { shared_memory_object::remove(shm_name); }
    // } remover;

    // Create shared memory segment for topic data
    boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create, g_shm_data_name.c_str(), 65535);

    g_segment_data = &segment;

    // Create shared memory segment for config data
    boost::interprocess::managed_shared_memory config_segment(boost::interprocess::open_or_create, config_shm_name.c_str(), 65535);

    // Allocate mmemory in shared memory segment for the size of the type of ros1 message
    // void *ros1_data = segment.allocate(sizeof(ros1_msg_type));

    // Create storage for ROS1 publisher object as a variant
    shared_ptr<CRos1PublisherInterface> m_ros_if;

    // Create ros1 publisher object depending on the type of ros1 message
    if (ros1_msg_type == g_ros_msg_image)
    {
        std::shared_ptr<CRos1Publisher<::sensor_msgs::Image>> ros1_pub = make_shared<CRos1Publisher<::sensor_msgs::Image>>();

        {
            // lock mutex using lock_guard
            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mutex);

            // Get ros1 publisher config data depending of the type of ros1 message
            if (ros1_msg_type == g_ros_msg_image)
            {
                // Get ros1 publisher config data from shared memory
                CRosPublisherConfig<::sensor_msgs::Image> *ros1_pub_config = config_segment.find<CRosPublisherConfig<::sensor_msgs::Image>>(config_shm_name.c_str()).first;

                // Check if config data is valid
                if (ros1_pub_config == nullptr)
                {
                    std::cout << "Error: ros1 publisher config data not found" << std::endl;
                    return 1;
                }

                // Set callback functino in ros1 publisher config data
                try
                {
                    ros1_pub_config->pub_info.f_get_message = gen_ros1_msg_from_shm_data<::sensor_msgs::Image>;
                }
                catch (const std::exception &e)
                {
                    std::cerr << e.what() << '\n';
                    return 1;
                }

                // Set ros1 publisher config data
                ros1_pub->set_config(*ros1_pub_config);

                m_ros_if = std::static_pointer_cast<CRos1PublisherInterface>(ros1_pub);
            }
            else
            {
                throw std::runtime_error("Message type not supported currently");
            }

        } // Destroy lock_guard and unlock mutex

        // Initialize ros1 publisher
        m_ros_if->init();

        // Publish ros1 messages until some error occurred
        m_ros_if->publish_loop();

        // Check if there is some error in ros1
        if (m_ros_if->ros_error())
        {
            std::cout << "Error: ros1 publisher error" << std::endl;
            return 2;
        }
    }
    else
    {
        std::cout << "Error: ros1 message type not supported" << std::endl;
        return 1;
    }

    return 0;
}
