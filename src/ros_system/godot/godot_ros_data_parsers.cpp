
#include "godot_ros_data_parsers.hpp"

#include "ros/ros.h"

/** Map godot image format to ROS image format
 *
 */
static std::map<Image::Format, std::string> g_map_godot_img_format_to_ros_img_format{
    {Image::Format::FORMAT_RGB8, "rgb8"},
    {Image::Format::FORMAT_RGBA8, "rgba8"},
    {Image::Format::FORMAT_RGB8, "rgb8"}};

void godotImgtoROSImg(uint64_t f_seq_id, uint64_t f_frame_id, ::godot::Image &f_in, ::sensor_msgs::Image &f_out)
{

    fout.header.seq = f_seq_id;
    fout.header.stamp =  ros::Time::now();
    fout.header.frame_id = f_frame_id;
    // Convert godot image to grpc imageMsg
    f_out.height = f_in.get_height();
    f_out.witdh f_in.get_width();
    f_out.encoding = g_map_godot_img_format_to_ros_img_format[m_image_buf.read_buf->get_format()];
    f_out.is_bigendian = false;
    // l_out_img.set_step(m_image_buf.read_buf->get_width() * 3 * sizeof(uint8_t));

    // Set data
    ::godot::PackedByteArray l_data = f_in->get_data();

    auto l_num_data = l_data.size();

    l_out_img.step = l_num_data / l_out_img.height();

    // Resize out array
    f_out.data.resize(l_num_data);

    // std::memcpy(&m_msg->data[0], img->get_data().ptrw(), img->get_data().size());
    for (int64_t i_idx = 0; i_idx < l_num_data; i_idx++)
    {
        f_out.data[i_idx] = l_data[i_idx]);
    }

    // out height
    auto l_out_height = l_out_img.height();
    // out width
    auto l_out_width = l_out_img.width();
    // out data size
    auto l_out_data_size = l_out_img.uint8_data_size();

    // Print generated image height, withd and num of data
    cout << "+++++ Callback 1 Image height : " << l_out_img.height() << ", width : " << l_out_img.width() << ", array data size " << l_out_img.uint8_data_size() << endl;


}