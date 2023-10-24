
#pragma once

#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/camera3d.hpp"
#include "godot_cpp/classes/window.hpp"
#include "godot_cpp/classes/viewport.hpp"
#include "godot_cpp/classes/ref.hpp"
#include "godot_cpp/classes/image.hpp"
#include <godot_cpp/variant/variant.hpp>
#include "godot_cpp/classes/image.hpp"

#include <map>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <array>

#include "sensor_if/sensor_if.hpp"

namespace godot
{

    struct CSDFPose
    {
        float x = 0.f;
        float y = 0.f;
        float z = 0.f;
        float roll = 0.f;
        float pitch = 0.f;
        float yaw = 0.f;

        std::string relative_to = "";
    };

    struct CSensorBasicCameraConfig
    // Camera parameters
    {
        CBaseSensorConfig<::godot_grpc::simple_camera_service::imageMsg> base_sensor_config;

        // Camera name
        std::string cam_name;

        // Flag indicates if the camera is triggered by ROS topic
        bool triggered;

        // Name of the topic where to publish the camera frames
        std::string m_topic_name;

        std::string camera_info_topic;

        // Trigger topic
        std::string trigger_topic;

        float horizontal_fov = 1.0469999999999999f;

        // Image size
        uint32_t width = 320;
        uint32_t height = 240;

        // Vieport size
        Vector2 m_viewport_size;

        // Format
        Image::Format m_format;

        // Anti-aliasing
        uint16_t anti_aliasing = 4u;

        struct
        {
            // clip near
            float near_m_f32 = 0.1f;

            // clip far
            float far_m_f32 = 100.f;
        } clip;

        struct
        {
            bool enabled = false;
            std::string path = ".";
        } save;

        struct
        {
            bool enabled = false;
            std::string output = "depths";

            struct
            {
                float near_m_f32 = 0.1f;
                float far_m_f32 = 10.f;
            } clip;

        } depth_camera;

        std::string segmentation_type = "semantic";

        std::string box_type = "2d";

        struct
        {
            bool enabled = false;
            std::string type = "gaussian";
            float mean = 0.f;
            float stddev = 0.0f;

        } noise;

        struct
        {
            bool enabled = false;
            float k1 = 0.f;
            float k2 = 0.f;
            float k3 = 0.f;
            float p1 = 0.f;
            float p2 = 0.f;

            Vector2 center{0.5f, 0.5f};

        } distortion;

        struct
        {
            bool enabled = false;

            std::string type = "stereographic";
            bool scale_to_hfov = true;

            // Accordin to SDF 1.7 format custom_function mapping is defined by r=c1*f*fun(theta/c2 + c3)

            struct
            {
                bool enabled = false;

                float c1 = 1.f;
                float c2 = 1.f;
                float c3 = 0.f;
                float f = 1.f;
                std::string fun = "tan";

                float operator()(float f_theta)
                {
                    float l_ret_f32 = 0.f;
                    if (fun == "tan")
                    {
                        l_ret_f32 = c1 * f * tan(f_theta / c2 + c3);
                    }
                    else if (fun == "sin")
                    {
                        l_ret_f32 = c1 * f * sin(f_theta / c2 + c3);
                    }
                    else if (fun == "id")
                    {
                        // Identity function assumed ??
                        l_ret_f32 = c1 * f * (f_theta / c2 + c3);
                    }
                    else
                    {
                        throw std::runtime_error("Unknown function type to use in camera lens");
                    }

                    return l_ret_f32;
                }

            } custom_function;

            float cutoff_angle_rad = 1.5707f;

            uint16_t env_texture_size = 256;

            struct
            {
                bool enabled = false;

                float fx_pixel = 277.f;
                float fy_pixel = 277.f;
                float cx_pixel = 160.f;
                float cy_pixel = 120.f;
                float s = 0.f;

            } intrinsics;

            struct
            {
                bool enabled = false;

                float p_fx_pixel = 277;
                float p_fy_pixel = 277;
                float p_cx_pixel = 160;
                float p_cy_pixel = 120;
                float tx_pixel = 0;
                float ty_pixel = 0;

            } projection;

        } lens;

        uint32_t visibility_mask = 0xffffffff;
        std::string optical_frame_id = "";

        CSDFPose pose;
    };

    /**
     * @class CSensorBasicCamera
     *
     * @brief A class representing a basic camera sensor in a 3D environment.
     *
     * This class inherits from the Camera3D class and the CBaseSensor class for
     * sensor_msgs::Image data.
     * As the class derives from ::godot::Camera3D all the properties of this class
     * will be avaliable in the Godot editor.
     *
     * The image is generated each time CSensorBasicCamera::gen_image_callback is
     * called as a callback from the CBaseSensor ros publisher
     *
     */
    class CSensorBasicCamera : public Camera3D
    {

        GDCLASS(CSensorBasicCamera, Camera3D)

    protected:
        /**
         * @brief Method used by Godot system to register methods and parameters exposed to Godot
         *
         * The methods and vars exposed to Godot will be accesible from GDScript and the
         * parameters will be visible in the Godot UI editor
         */
        static void _bind_methods();

    public:
        CSensorBasicCamera();
        ~CSensorBasicCamera();

        /**
         * @brief Godot methot called for this node to initialize it
         *
         * When instanciating a scene Godot will instantiate nodes down the tree making _init() calls
         *  and build the tree going downwards from the root
         */
        void _init();

        /**
         * @brief Godot method called for this node to process it each visual frame
         *
         * This is the method where the developer is supossed to program the node
         * logic for each frame, is called each frame before rendering
         */
        void _process(float delta);

        /**
         * @brief Godot method called just one time, before the first frame is rendered. Used to initialize the node logic
         *
         */
        void _ready() override;

        void _notification(int f_notification);

        GDROS1PUB_DECLARE_METHODS

        void set_ros_init_node_props(bool f_value);

        bool get_ros_init_node_props();

    private:

        //Sensor interface object
        std::shared_ptr<CSensorIf<::godot::Image>> m_sensor_if;

        CSensorIfConfig<::godot::Image> m_sensor_if_config;

        // Config
        CSensorBasicCameraConfig m_camera_config;

        bool m_aux_init_props = false;

        bool m_aux_first_call_init = true;

        /**
         * Flag to check if initialized flag has been set
         */
        bool m_initialized = false;

        // std::mutex m_mtx_init;

        ::godot::Viewport* m_viewport;


    };

}