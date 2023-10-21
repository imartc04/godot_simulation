
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

using namespace std;
using namespace godot;

GDROS1PUB_DEF_METHODS(CSensorBasicCamera, (*this), ::sensor_msgs::Image)

GRPC_DEFINE_METHODS(CSensorBasicCamera, m_camera_config.base_sensor_config.grpc_subprocess_config)

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
    //std::this_thread::sleep_for(10ms);


    // Check if class has  been initialized
    if (ros_manager_initialized())
    {

        if (!m_initialized)
        {
            // Set initialized
            m_initialized = true;

            t_baseSensor::init();
        }

        // Print inside get_initialized
        cout << "Inside get_initialized" << endl;

        auto l_img2 = get_viewport()->get_texture()->get_image();

        m_image_buf.mutex.lock();

        m_image_buf.write_buf = get_viewport()->get_texture()->get_image();
        m_image_buf.seq_id++;

        // print write buff image height, width and num of data
        cout << "Write buffer Image height : " << m_image_buf.write_buf->get_height() << ", width : " << m_image_buf.write_buf->get_width() << ", array data size " << m_image_buf.write_buf->get_data().size() << endl;

        // Print imgage height and width
        // cout << "Img2 height : " << l_img2->get_height() << ", width : " << l_img2->get_width() << ", array data size " << l_img2->get_data().size() << endl;

        m_image_buf.mutex.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(1.f / m_camera_config.base_sensor_config.ros1_pub_config.pub_rate_hz_f32() * 1000.f * 0.5f)));
    }
  
}

// void CSensorBasicCamera::parse_godot_out_image(uint64_t f_seq_id)
// {

//     auto &l_out_img = m_image_buf.out_image;

//     // Convert godot image to grpc imageMsg
//     l_out_img.set_height(m_image_buf.read_buf->get_height());
//     l_out_img.set_width(m_image_buf.read_buf->get_width());
//     l_out_img.set_encoding(m_map_godot_img_format_to_ros_img_format[m_image_buf.read_buf->get_format()]);
//     l_out_img.set_is_bigendian(false);
//     // l_out_img.set_step(m_image_buf.read_buf->get_width() * 3 * sizeof(uint8_t));

//     // Set data
//     ::godot::PackedByteArray l_data = m_image_buf.read_buf->get_data();

//     auto l_num_data = l_data.size();

//     l_out_img.set_step(l_num_data / l_out_img.height());

//     // Resize out array
//     l_out_img.mutable_uint8_data()->Resize(l_num_data, 0u);

//     // std::memcpy(&m_msg->data[0], img->get_data().ptrw(), img->get_data().size());
//     for (int64_t i_idx = 0; i_idx < l_num_data; i_idx++)
//     {
//         l_out_img.set_uint8_data(i_idx, l_data[i_idx]);
//     }

//     // out height
//     auto l_out_height = l_out_img.height();
//     // out width
//     auto l_out_width = l_out_img.width();
//     // out data size
//     auto l_out_data_size = l_out_img.uint8_data_size();

//     // Print generated image height, withd and num of data
//     cout << "+++++ Callback 1 Image height : " << l_out_img.height() << ", width : " << l_out_img.width() << ", array data size " << l_out_img.uint8_data_size() << endl;

//     // Set frame id
//     l_out_img.set_frame_id(f_seq_id);
// }

::godot_grpc::simple_camera_service::imageMsg CSensorBasicCamera::gen_image_callback()
{

    uint64_t l_seq_id = 0;

    // Change buffers to get new image
    m_image_buf.mutex.lock();

    // Swap buffers
    std::swap(m_image_buf.read_buf, m_image_buf.write_buf);

    // Get seq_id
    l_seq_id = m_image_buf.seq_id;

    // unlock
    m_image_buf.mutex.unlock();

    // Convert image to grpc format
    parse_godot_out_image(l_seq_id);

    // Return copy of out image
    return m_image_buf.out_image;
}

void CSensorBasicCamera::set_config(CSensorBasicCameraConfig const &f_config)
{
    m_camera_config = f_config;
}

// Get config
CSensorBasicCameraConfig &CSensorBasicCamera::get_config()
{
    return m_camera_config;
}
