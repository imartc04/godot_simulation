#pragma once

#include "ros_interface/ros1/ros1_pub.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/process.hpp>
#include <random>
#include "grpc_interface/gen_protoc/simple_camera_service.grpc.pb.h"
// #include "grpc_interface/gen_protoc/simple_camera_service.h"

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

        // gRPC config
        struct
        {
            std::string server_address;
            std::string server_port;

        } grpc_config;

        /**
         * Callaback to generate new data
         */
        std::function<t_ros_msg()> new_data_callback;

        // Ros pub config
        CRos1PublisherConfig<t_ros_msg> ros1_pub_config;
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

            //Create gRPC server builder
            ServerBuilder builder;
            builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());

            // Create grpc server from config
            if (m_config.ros1_pub_config.proto_config.get_topic_type() == "sensor_msgs/Image")
            {
                auto grpc_server = std::make_shared<CSimpleCameraServiceServerImpl>(m_config.grpc_config.server_address, m_config.grpc_config.server_port);
                grpc_server->start();
            }
            else
            {
                std::cout << "Error: ros1 message type not supported" << std::endl;
                return;
            }

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