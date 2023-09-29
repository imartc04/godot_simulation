#pragma once

// #include "ros_interface/ros1/ros1_pub.hpp"
#include <boost/process.hpp>
#include <random>
#include "grpc_interface/gen_protoc/simple_camera_service.grpc.pb.h"
#include "grpc_interface/simple_camera_server_impl.hpp"
#include <grpcpp/client_context.h>
#include <exception>
#include <locale>

// #include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>

#include <memory>

#include <ros/ros.h> //ros time

// #include "grpc_interface/gen_protoc/simple_camera_service.h"

namespace godot
{

    namespace bp = ::boost::process;

    // Configuration struct
    template <typename t_data_gen>
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
        std::function<t_data_gen()> new_data_callback;

        // Ros pub config
        godot_grpc::ros1::ROS1PublisherConfig ros1_pub_config;
    };

    template <typename t_data_gen>
    class CBaseSensor
    {
    public:
        typedef CBaseSensorConfig<t_data_gen> t_config;
        typedef CBaseSensor<t_data_gen> t_baseSensor;

        CBaseSensor()
        {

            if (!::ros::Time::isValid())
            {
                ::ros::Time::init();
            }
        };
        ~CBaseSensor()
        {

            if (m_ros1_pub_thread.joinable())
            {
                m_ros1_pub_thread.joinable();
            }

            if (m_ros1_pub_process)
            {
                m_ros1_pub_process->terminate();
                m_ros1_pub_process->wait();
            }
        };

        void set_config(t_config const &f_config)
        {

            // Set config
            m_config = f_config;
        }

        t_config &get_config()
        {
            return m_config;
        }

        // Method to manage ROS1 publisher process
        void manage_ros1_pub_process(uint16_t f_status)
        {
            if (f_status == 1)
            {
                throw std::runtime_error("Error: ROS1 publisher process was unable to init due some configuration problem");
            }
            else if (f_status == 2)
            {

                create_new_subprocess();

                // Create new process
                // m_ros1_pub_process = std::make_unique<boost::process::child>(boost::process::search_path("ros1_pub_node"), boost::process::std_out > stdout, boost::process::std_err > stderr);
            }
            else
            {
                // Nothing to do
            }
        }

        /**
         * Initialize ROS 1 publsiher node realted data
         *
         * Create ROS 1 node as a process passing data to it by shared memory
         *
         */
        void init()
        {

            // Create thread to manage ROS 1 publisher process
            m_ros1_pub_thread = std::thread(&t_baseSensor::ros1_pub_thread, this);
        }

    protected:
        // ROS1 publisher
        t_config m_config;

    private:
        /**** VARIABLES ****/

        /**
         * Thread to create and manage ROS 1 publisher process
         */
        std::thread m_ros1_pub_thread;

        std::unique_ptr<::grpc::Server> m_grpc_server;

        std::unique_ptr<boost::process::child> m_ros1_pub_process;

        std::shared_ptr<SimpleCameraServerImpl> camera_server;

        /**** METHODS ****/

        void create_new_subprocess()
        {

            // kill process if needed

            if (m_ros1_pub_process)
            {
                m_ros1_pub_process->terminate();
            }

            auto env = boost::this_process::environment();

            // Access the PATH variable
            std::string pathVar = env["GODOT_GEN_BIN_DIR"].to_string();

            // Create a boost::process::child with the command line
            m_ros1_pub_process = std::make_unique<bp::child>(pathVar + "/ros1_pub_node", bp::args({m_config.grpc_config.server_address.c_str(), m_config.grpc_config.server_port.c_str()}), bp::std_out > stdout, bp::std_err > stderr, bp::std_in < stdin);
        }

        void ros1_pub_thread()
        {
            // Create gRPC server builder
            ::grpc::ServerBuilder builder;
            builder.AddListeningPort(m_config.grpc_config.server_address + ":" + m_config.grpc_config.server_port, grpc::InsecureServerCredentials());

            // Create grpc server from config
            if (m_config.ros1_pub_config.topic_type() == "sensor_msgs/Image")
            {
                camera_server = std::make_shared<SimpleCameraServerImpl>();

                builder.RegisterService(camera_server.get());

                // Set callback function to generate new image
                camera_server->set_data_callback(m_config.new_data_callback);

                // Set ROS1 publisher config
                camera_server->set_ros1_config(m_config.ros1_pub_config);

                // Set callback function to manage client status
                camera_server->set_client_status_callback(std::bind(&t_baseSensor::manage_ros1_pub_process, this, std::placeholders::_1));

                m_grpc_server = builder.BuildAndStart();

                // Create child process
                create_new_subprocess();
            }
            else
            {
                std::cout << "Error: ros1 message type not supported" << std::endl;

                // Throw error
                throw std::runtime_error("Error: ros1 message type not supported");
            }

            while (true)
            {
                // Server wait for new requests
                m_grpc_server->Wait();
            }
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