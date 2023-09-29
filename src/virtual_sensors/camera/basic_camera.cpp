
#include "basic_camera.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/viewport_texture.hpp"
#include <functional>
#include <algorithm>
#include <godot_cpp/classes/engine.hpp>
#include <iostream>
#include <chrono>
#include <thread>

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
            t_baseSensor::init();
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
    // Inittialize image buffers
    m_image_buf.read_buf = m_image_buf.data_buf[0];
    m_image_buf.write_buf = m_image_buf.data_buf[1];
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

// void CSensorBasicCamera::convert_image_to_ros_msg(Image *f_godot_img)
// {

//     // Create ROS msg
//     auto &l_ros_image_r = m_callback_flag.image_buffer;

//     // Set header
//     l_ros_image_r.header.stamp = ros::Time::now();
//     l_ros_image_r.header.seq = m_seg_id;

//     // Check for overflow in seq id
//     if (m_seg_id == UINT64_MAX)
//     {
//         m_seg_id = 0;
//     }
//     else
//     {
//         m_seg_id++;
//     }

//     l_ros_image_r.header.frame_id = std::to_string(l_ros_image_r.header.seq);

//     // Set image size
//     l_ros_image_r.height = f_godot_img->get_height();
//     l_ros_image_r.width = f_godot_img->get_width();

//     // Set encoding
//     l_ros_image_r.encoding = m_map_godot_img_format_to_ros_img_format[f_godot_img->get_format()];

//     // set big endian to false
//     l_ros_image_r.is_bigendian = false;

//     // Set step, all godot formats are 8 bits per channel
//     l_ros_image_r.step = f_godot_img->get_width() * 3 * sizeof(uint8_t);

//     // Set data

//     cout << "Image format godot : " << f_godot_img->get_format() << endl;
//     cout << "Image height : " << f_godot_img->get_height() << ", width : " << f_godot_img->get_width() << ", array data size " << f_godot_img->get_data().size() << endl;

//     // Get image data array
//     auto l_data = f_godot_img->get_data();

//     // Resize out array
//     m_callback_flag.image_buffer.data.resize(l_data.size());

//     std::memcpy(&l_ros_image_r.data[0], l_data.ptr(), sizeof(uint8_t) * l_data.size());
// }

void CSensorBasicCamera::_process(float delta)
{

   // set_current(true);

    get_viewport()->get_texture()->get_image()->save_png("/tfm/test_cpp_camera/test.png");

    //std::this_thread::sleep_for(40ms);
    
    // l_img->convert(::godot::Image::Format::FORMAT_RGB8);
    // l_img->save_png("/tfm/test_camera.png");

    // Check if class has  been initialized
    if (get_initialized())
    {

        m_image_buf.mutex.lock();

        m_image_buf.write_buf = get_viewport()->get_texture()->get_image();
        m_image_buf.seq_id++;

        m_image_buf.mutex.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(1.f / m_camera_config.base_sensor_config.ros1_pub_config.pub_rate_hz_f32() * 1000.f * 0.5f)));
    }
}

void CSensorBasicCamera::parse_godot_out_image(uint64_t f_seq_id)
{

    auto &l_out_img = m_image_buf.out_image;



//  m_msg->step = img->get_data().size() / m_msg->height;
//     m_msg->data.resize(img->get_data().size());
//     // TODO(flynneva): optimize this / find a better way
//     std::memcpy(&m_msg->data[0], img->get_data().ptrw(), img->get_data().size());


    // Convert godot image to grpc imageMsg
    l_out_img.set_height(m_image_buf.read_buf->get_height());
    l_out_img.set_width(m_image_buf.read_buf->get_width());
    l_out_img.set_encoding(m_map_godot_img_format_to_ros_img_format[m_image_buf.read_buf->get_format()]);
    l_out_img.set_is_bigendian(false);
    // l_out_img.set_step(m_image_buf.read_buf->get_width() * 3 * sizeof(uint8_t));
    

    // Set data
    ::godot::PackedByteArray l_data = m_image_buf.read_buf->get_data();

    auto l_num_data = l_data.size();

    l_out_img.set_step(l_num_data / l_out_img.height());

    // Resize out array
    l_out_img.mutable_uint8_data()->Resize(l_num_data, 0u);

// std::memcpy(&m_msg->data[0], img->get_data().ptrw(), img->get_data().size());
    for (int64_t i_idx = 0; i_idx < l_num_data; i_idx++)
    {
        l_out_img.set_uint8_data(i_idx, l_data[i_idx]);
    }

    // Set frame id
    l_out_img.set_frame_id(f_seq_id);
}

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
