

#pragma once

#include <cstdint>
#include <godot_cpp/classes/image.hpp>

/**
 * Wrapper structure used to parse generated godot image to ROS one
 */
struct CGodotImgROS
{
    uint64_t seq_id;
    uint64_t frame_id;
    ::godot::Imageg godot_image;
};
