#pragma once

#include "ros_interface/ros1/ros1_pub.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/process.hpp>
#include <random>

namespace godot
{

    // Configuration struct
    template <typename t_ros_msg>
    struct CBaseSensorConfig
    {

        // Sensor name
        std::string name;

        // Sensor type
        std::string type;

        // Associated URDF link
        std::string link;

        // Always on
        bool always_on;

        // update rate
        float update_rate;
        // flag update rate
        bool update_rate_flag;

        bool visualize;

        bool enable_metrics;

        struct
        {
            float x = 0.f;
            float y = 0.f;
            float z = 0.f;
            float roll = 0.f;
            float pitch = 0.f;
            float yaw = 0.f;

            std::string ref_frame;

        } pose;

        // Ros pub config
        CRosPublisherConfig<t_ros_msg> ros1_pub_config;
    };

    template <typename t_ros_msg>
    class CBaseSensor
    {

    public:
        CBaseSensor()
        {

            // Check if ROS is intialized and initialize it if not
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "godot_ros_sensor"); //, ros::init_options::NoSigintHandler
                static ros::NodeHandle nh;
                // ros::start();
            }
        };
        ~CBaseSensor(){};

        void set_config(CBaseSensorConfig<t_ros_msg> const &f_config)
        {

            // Set config
            m_config = f_config;
        }

        CBaseSensorConfig<t_ros_msg> &get_config()
        {
            return m_config;
        }

        /**
         * Method to write data to shared memory
         * for the ROS 1 node to publish it
         */
        void write_data_to_shm(t_ros_msg const &f_data)
        {

            // lock mutex using lock_guard
            boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex);

            // Write data into shared memory
            *m_data_obj_shm = f_data;
        }

        /**
         * Initialize ROS 1 publsiher node realted data
         *
         * Create ROS 1 node as a process passing data to it by shared memory
         *
         */
        void init()
        {

            // Create named shared data string
            auto shm_data_name = gen_string_shm();

            // Create named config for shared data string
            auto shm_config_name = gen_string_shm();

            // Create named mutex name string
            auto shm_mutex_name = "mutex_" + gen_string_shm();

            // Create mutex
            mutex = std::make_unique<boost::interprocess::named_mutex>(boost::interprocess::open_or_create, shm_mutex_name.c_str());

            auto auxmem = boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, shm_data_name.c_str(), sizeof(t_ros_msg), nullptr, boost::interprocess::read_write);

            // Crate shared memory segment
            m_segment = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_or_create, shm_data_name.c_str(), sizeof(t_ros_msg));

            // Construct data shared memory object
            m_data_obj_shm = m_segment->construct<t_ros_msg>(shm_data_name.c_str())();

            // Create shared memory object for config
            m_config_segment = std::make_unique<boost::interprocess::managed_shared_memory>(boost::interprocess::open_or_create, shm_config_name.c_str(), sizeof(CBaseSensorConfig<t_ros_msg>));

            // Set the value of the shared memory config object to the config object
            m_config_obj_shm = m_config_segment->construct<CBaseSensorConfig<t_ros_msg>>(shm_config_name.c_str())(m_config);

            // Create thread to manage ROS 1 publisher process
            m_ros1_pub_thread = std::thread(&CBaseSensor<t_ros_msg>::ros1_pub_thread, this);
        }

    protected:
        // ROS1 publisher
        CBaseSensorConfig<t_ros_msg> m_config;

        CBaseSensorConfig<t_ros_msg> *m_config_obj_shm;

        t_ros_msg *m_data_obj_shm;

    private:
        /**** VARIABLES ****/
        /**
         * ROS 1 publisher mutex object
         * to manage access to shared memory
         */
        std::unique_ptr<boost::interprocess::named_mutex> mutex;

        /**
         * ROS 1 publisher shared memory for data
         */
        std::unique_ptr<boost::interprocess::managed_shared_memory> m_segment;

        /**
         * Shared memory object for config data
         */
        std::unique_ptr<boost::interprocess::managed_shared_memory> m_config_segment;

        /**
         * Thread to create and manage ROS 1 publisher process
         */
        std::thread m_ros1_pub_thread;

        /**** METHODS ****/

        void ros1_pub_thread()
        {
            // Create ros node subprocess
            int l_process_res = 0;
            do
            {

                // Generate random strings for shared memory
                auto shm_data_name = gen_string_shm();
                auto shm_config_name = "config_" + gen_string_shm();
                auto shm_mutex_name = "mutex_" + gen_string_shm();
                auto ros_node_name = "ros_node_" + m_config.ros1_pub_config.node_name + gen_string_shm();

                boost::process::child ros_node_subprocess(boost::process::search_path("./ros1_pub_node"), m_config.ros1_pub_config.topic_type, shm_data_name, shm_config_name, shm_mutex_name, ros_node_name);

                if (!ros_node_subprocess.valid())
                {
                    throw std::runtime_error("Error creating ros node subprocess");
                }

                // Wait for ros node subprocess to finish and get result of execution
                ros_node_subprocess.wait();

                l_process_res = ros_node_subprocess.exit_code();

            } while (l_process_res == 2);
        }

        /**
         * Mehod to obtain a random generated string value to name interprocess resoureces
         *
         */
        std::string gen_string_shm(uint16_t f_lengt = 10)
        {

            // Random string
            std::string random_string;

            // Random string length
            int random_string_length = f_lengt;

            // Random string characters
            std::string random_string_characters = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

            // Random device
            std::random_device random_device;

            // Random engine
            std::mt19937 random_engine(random_device());

            // Random distribution
            std::uniform_int_distribution<> distribution(0, random_string_characters.size() - 1);

            // Generate random string
            for (int i = 0; i < random_string_length; i++)
            {
                random_string += random_string_characters[distribution(random_engine)];
            }

            return "godot_ros_sensor_" + random_string;
        };
    };

}