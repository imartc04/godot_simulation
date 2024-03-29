
#include "basic_camera.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/viewport_texture.hpp"
#include <functional>
#include <algorithm>
#include <godot_cpp/classes/engine.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include "ros_manager/ros_manager.hpp"
#include "ros_system/godot/godot_ros_lib/ros_lib.hpp"

using namespace std;
using namespace godot;

GDROS1PUB_DEF_METHODS(CSensorBasicCamera, m_sensor_if_config.ros_pub_config)

void CSensorBasicCamera::_bind_methods()
{

    GDROS1PUB_BIND_METHODS(CSensorBasicCamera)
}

void CSensorBasicCamera::_notification(int f_notification)
{

    switch (f_notification)
    {
    case Node3D::NOTIFICATION_ENTER_WORLD:
    {
        int aux = 10;
        break;
    }
    case Node3D::NOTIFICATION_ENTER_TREE:
    {
        int aux = 10;
        break;
    }

    default:
    {
        break;
    }
    };
}

CSensorBasicCamera::CSensorBasicCamera()
{
    // Inittialize image buffers
    // m_image_buf.read_buf = m_image_buf.data_buf[0];
    // m_image_buf.write_buf = m_image_buf.data_buf[1];
}

CSensorBasicCamera::~CSensorBasicCamera()
{
}

void CSensorBasicCamera::_init()
{
}

// Implement _ready
void CSensorBasicCamera::_ready()
{
    // Set config callbak
    m_camera_config.base_sensor_config.new_data_callback = std::bind(&CSensorBasicCamera::gen_image_callback, this);

    m_camera_config.base_sensor_config.ros1_pub_config.set_enabled(true);
    m_camera_config.base_sensor_config.ros1_pub_config.set_pub_type(::godot_grpc::ros1::ROS1PublishType::PUBLISH_TYPE_RATE);

    m_camera_config.base_sensor_config.ros1_pub_config.set_topic_type("sensor_msgs/Image");

    // Configure ROS1 publisher
    t_baseSensor::set_config(m_camera_config.base_sensor_config);

    m_image_buf.write_buf = get_viewport()->get_texture()->get_image();
    m_image_buf.read_buf = get_viewport()->get_texture()->get_image();

    // Create Viewport object
    // m_viewport = memnew(Viewport);

    // // Get current parent
    // auto l_curr_parent = get_parent();

    // // Set viewport parent
    // l_curr_parent->add_child(m_viewport);
    // // l_curr_parent->call_deferred("add_child", viewport);

    // // Set new parent of camera the viewpor object
    // reparent(m_viewport);
    // this->call_deferred("reparent", viewport);
}

void CSensorBasicCamera::_process(float delta)
{

    // set_current(true);
    // std::this_thread::sleep_for(10ms);

    // Check if class has  been initialized
    if (ros_manager_initialized())
    {

        if (!m_initialized)
        {
            // Set initialized
            m_initialized = true;

            m_sensor_if_config.ros_pub_gen_func = std::bind(createImagePub, std::placeholders::_1);

            // Init sensor interface
            m_sensor_if.set_config(m_sensor_if_config);

            m_sensor_if.init();
        }

        // Print inside get_initialized
        cout << "Inside get_initialized" << endl;

        m_sensor_if->update_data(*get_viewport()->get_texture()->get_image());

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(1.f / m_camera_config.base_sensor_config.ros1_pub_config.pub_rate_hz_f32() * 1000.f * 0.5f)));
    }
}
