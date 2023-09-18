
#include "basic_camera.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/viewport_texture.hpp"
#include <functional>
#include <algorithm>
#include <godot_cpp/classes/engine.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace godot;

GDROS1PUB_DEF_METHODS(CSensorBasicCamera, (*this), ::sensor_msgs::Image)

void CSensorBasicCamera::_bind_methods()
{

    GDROS1PUB_BIND_METHODS(CSensorBasicCamera)

    //    ADD_PROPERTY(PropertyInfo(Variant::STRING, "ros1_node_name"), "set_ros1_node_name", "get_ros1_node_name");
    ClassDB::bind_method(D_METHOD("set_ros_init_node_props", "ros_init_node_props"), &CSensorBasicCamera::set_ros_init_node_props);
    ClassDB::bind_method(D_METHOD("get_ros_init_node_props"), &CSensorBasicCamera::get_ros_init_node_props);

    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ros_init_node_props"), "set_ros_init_node_props", "get_ros_init_node_props");
}

void CSensorBasicCamera::set_ros_init_node_props(bool f_value)
{
    if (m_aux_first_call_init)
    {
        m_aux_first_call_init = false;
    }
    else
    {
        if (!get_initialized())
        {
            CBaseSensor<::sensor_msgs::Image>::init();
            set_initialized(true);

            cout << "Camera sensor initialized, the process cannot be undo so if you need to initialize again rerun godot" << endl;
        }
    }

    m_aux_init_props = f_value;
}

bool CSensorBasicCamera::get_ros_init_node_props()
{
    return m_aux_init_props;
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
    // m_camera_config.base_sensor_config.ros1_pub_config.pub_info.f_get_message = std::bind(&CSensorBasicCamera::gen_image_callback, this);

    m_camera_config.base_sensor_config.ros1_pub_config.enabled = true;
    m_camera_config.base_sensor_config.ros1_pub_config.pub_info.pub_type = EPublishType::PUBLISH_TYPE_RATE;

    // Configure ROS1 publisher
    CBaseSensor<::sensor_msgs::Image>::set_config(m_camera_config.base_sensor_config);

    // Create Viewport object
    auto viewport = memnew(Viewport);

    // Get current parent
    auto l_curr_parent = get_parent();

    // Set viewport parent
    l_curr_parent->add_child(viewport);
    // l_curr_parent->call_deferred("add_child", viewport);

    // Set new parent of camera the viewpor object
    reparent(viewport);
    // this->call_deferred("reparent", viewport);
}

void CSensorBasicCamera::set_initialized(bool f_value)
{
    // lock mutex
    std::unique_lock<std::mutex> lock(m_mtx_init);

    m_initialized = f_value;
}

bool CSensorBasicCamera::get_initialized()
{
    // lock mutex
    std::unique_lock<std::mutex> lock(m_mtx_init);
    return m_initialized;
}

void CSensorBasicCamera::convert_image_to_ros_msg(Image *f_godot_img)
{

    // Create ROS msg
    auto &l_ros_image_r = m_callback_flag.image_buffer;

    // Set header
    l_ros_image_r.header.stamp = ros::Time::now();
    l_ros_image_r.header.seq = m_seg_id;

    // Check for overflow in seq id
    if (m_seg_id == UINT64_MAX)
    {
        m_seg_id = 0;
    }
    else
    {
        m_seg_id++;
    }

    l_ros_image_r.header.frame_id = std::to_string(l_ros_image_r.header.seq);

    // Set image size
    l_ros_image_r.height = f_godot_img->get_height();
    l_ros_image_r.width = f_godot_img->get_width();

    // Set encoding
    l_ros_image_r.encoding = m_map_godot_img_format_to_ros_img_format[f_godot_img->get_format()];

    // set big endian to false
    l_ros_image_r.is_bigendian = false;

    // Set step, all godot formats are 8 bits per channel
    l_ros_image_r.step = f_godot_img->get_width() * 3 * sizeof(uint8_t);

    // Set data

    cout << "Image format godot : " << f_godot_img->get_format() << endl;
    cout << "Image height : " << f_godot_img->get_height() << ", width : " << f_godot_img->get_width() << ", array data size " << f_godot_img->get_data().size() << endl;

    // Get image data array
    auto l_data = f_godot_img->get_data();

    // Resize out array
    m_callback_flag.image_buffer.data.resize(l_data.size());

    std::memcpy(&l_ros_image_r.data[0], l_data.ptr(), sizeof(uint8_t) * l_data.size());
}

void CSensorBasicCamera::_process(float delta)
{

    // Check if class has  been initialized
    if (get_initialized())
    {

        auto l_image = get_viewport()->get_texture()->get_image();

        {
            // std::unique_lock<std::mutex> l_lock(m_callback_flag.mutex);

            // Convert image to ROS msg
            convert_image_to_ros_msg(*l_image);

            // Call base sensor write data to shm
            CBaseSensor<::sensor_msgs::Image>::write_data_to_shm(m_callback_flag.image_buffer);

            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(1.f / m_camera_config.base_sensor_config.ros1_pub_config.pub_info.pub_rate_hz_f32 * 1000.f * 0.5f)));

        } // Delete lock
    }
}

void CSensorBasicCamera::set_new_image_ready_flag(bool f_flag_b)
{
    std::unique_lock<std::mutex> lock(m_callback_flag.mutex);
    m_callback_flag.new_image_ready_b = f_flag_b;
}

bool CSensorBasicCamera::get_new_image_ready_flag()
{
    std::unique_lock<std::mutex> lock(m_callback_flag.mutex);
    return m_callback_flag.new_image_ready_b;
}

void CSensorBasicCamera::set_pub_image_flag(bool f_flag_b)
{
    std::unique_lock<std::mutex> lock(m_callback_flag.mutex);
    m_callback_flag.pub_image_b = f_flag_b;
}

bool CSensorBasicCamera::get_pub_image_flag()
{
    std::unique_lock<std::mutex> lock(m_callback_flag.mutex);
    return m_callback_flag.pub_image_b;
}

::sensor_msgs::Image CSensorBasicCamera::gen_image_callback()
{

    // Mark flag to obtain image
    set_pub_image_flag(true);

    // Wait for new image generated
    ::sensor_msgs::Image l_out_image;

    // {
    //     std::unique_lock<std::mutex> lock(m_callback_flag.mutex);
    //     m_callback_flag.cond_var.wait(lock, [this]
    //                                   { return m_callback_flag.new_image_ready_b; });

    //     // Copy image
    //     l_out_image = m_callback_flag.image_buffer;

    //     // Reset flag
    //     m_callback_flag.new_image_ready_b = false;

    // } // unlock mutex

    return std::move(l_out_image);
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
