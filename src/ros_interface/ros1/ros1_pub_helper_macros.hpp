#pragma once
/** \file ros1_pub.hpp
 * @brief Contains ROS1 publisher auxiliar macros
 * This file define macros because ros publisher needs config parameters that have to be allowed to be set from Godot UI
 * and by using a macro we dont have to write the same code in each class that uses the CRos1Publisher
 */

/**
 * Macro to allow faster godot bind method creation
 *
 * This macro is intended to be used in the .cpp file of the class that will use the CRos1Publisher class
 * For a example of usage see the CSensorBasicCamera class
 *
 * @param CLASS - Name of the GDClass
 * @param MEMBER_NAME - Name of the CRos1Publisher member within the CLASS
 * @param ROS1_MSG_TYPE - ROS1 message type
 *
 * @see CSensorBasicCamera
 */
#define GDROS1PUB_DEF_METHODS(CLASS, MEMBER_NAME, ROS1_MSG_TYPE)                                                                            \
    void CLASS::set_ros1_node_name(String f_node_name)                                                                                      \
    {                                                                                                                                       \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.set_node_name(f_node_name.utf8().get_data());                           \
    }                                                                                                                                       \
                                                                                                                                            \
    String CLASS::get_ros1_node_name()                                                                                                      \
    {                                                                                                                                       \
        return MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.node_name().c_str();                                             \
    }                                                                                                                                       \
                                                                                                                                            \
    void CLASS::set_ros1_topic_name(String f_topic_name)                                                                                    \
    {                                                                                                                                       \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.set_topic_name(f_topic_name.utf8().get_data());                         \
    }                                                                                                                                       \
    String CLASS::get_ros1_topic_name()                                                                                                     \
    {                                                                                                                                       \
        return MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.topic_name().c_str();                                            \
    }                                                                                                                                       \
    void CLASS::set_ros1_queue_size(int f_queue_size)                                                                                       \
    {                                                                                                                                       \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.set_queue_size(f_queue_size);                                           \
    }                                                                                                                                       \
    int CLASS::get_ros1_queue_size()                                                                                                        \
    {                                                                                                                                       \
        return MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.queue_size();                                                    \
    }                                                                                                                                       \
    void CLASS::set_ros1_pub_rate(float f_pub_rate_hz)                                                                                      \
    {                                                                                                                                       \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.set_pub_rate_hz_f32(f_pub_rate_hz);                                     \
    }                                                                                                                                       \
    float CLASS::get_ros1_pub_rate()                                                                                                        \
    {                                                                                                                                       \
        return MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.pub_rate_hz_f32();                                               \
    }                                                                                                                                       \
    void CLASS::set_ros1_pub_type_rate(int f_type)                                                                                          \
    {                                                                                                                                       \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.set_pub_type(static_cast<::godot_grpc::ros1::ROS1PublishType>(f_type)); \
    }                                                                                                                                       \
    int CLASS::get_ros1_pub_type_rate()                                                                                                     \
    {                                                                                                                                       \
        return static_cast<int>(MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.pub_type());                                    \
    }                                                                           

/**
 * Macro to allow faster godot bind method creation
 *
 * This macro is intended to be used in the .hpp file of the class that will use the CRos1Publisher class
 * For a example of usage see the header file of CSensorBasicCamera class
 *
 * @see CSensorBasicCamera
 */
#define GDROS1PUB_DECLARE_METHODS                            \
    void set_ros1_node_name(String f_node_name);               \
    String get_ros1_node_name();                               \
    void set_ros1_topic_name(String f_topic_name);             \
    String get_ros1_topic_name();                              \
    void set_ros1_queue_size(int f_queue_size);                \
    int get_ros1_queue_size();                                 \
    void set_ros1_pub_rate(float f_pub_rate_hz);               \
    float get_ros1_pub_rate();                                 \
    void set_ros1_pub_type_rate(int f_type);                   \
    int get_ros1_pub_type_rate();                              
   

/**
 * Macro to allow faster godot bind method creation
 *
 * This macro is intended to be used in the implementation of the _bind_methods method of the class node
 * that use the CRos1Publisher class. This will bind to the godot system the methods defined in the macro
 * #GDROS1PUB_DECLARE_METHODS
 * For a example of usage see the header file of CSensorBasicCamera class
 *
 * @see CSensorBasicCamera
 */
#define GDROS1PUB_BIND_METHODS(CLASS)                                                                               \
    ClassDB::bind_method(D_METHOD("set_ros1_node_name", "ros1_node_name"), &CLASS::set_ros1_node_name);             \
    ClassDB::bind_method(D_METHOD("get_ros1_node_name"), &CLASS::get_ros1_node_name);                               \
    ClassDB::bind_method(D_METHOD("set_ros1_topic_name", "ros1_topic_name"), &CLASS::set_ros1_topic_name);          \
    ClassDB::bind_method(D_METHOD("get_ros1_topic_name"), &CLASS::get_ros1_topic_name);                             \
    ClassDB::bind_method(D_METHOD("set_ros1_queue_size", "ros1_queue_size"), &CLASS::set_ros1_queue_size);          \
    ClassDB::bind_method(D_METHOD("get_ros1_queue_size"), &CLASS::get_ros1_queue_size);                             \
    ClassDB::bind_method(D_METHOD("set_ros1_pub_rate", "ros1_pub_rate"), &CLASS::set_ros1_pub_rate);                \
    ClassDB::bind_method(D_METHOD("get_ros1_pub_rate"), &CLASS::get_ros1_pub_rate);                                 \
    ClassDB::bind_method(D_METHOD("set_ros1_pub_type_rate", "ros1_pub_type_rate"), &CLASS::set_ros1_pub_type_rate); \
    ClassDB::bind_method(D_METHOD("get_ros1_pub_type_rate"), &CLASS::get_ros1_pub_type_rate);                       \
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "ros1_node_name"), "set_ros1_node_name", "get_ros1_node_name");      \
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "ros1_topic_name"), "set_ros1_topic_name", "get_ros1_topic_name");   \
    ADD_PROPERTY(PropertyInfo(Variant::INT, "ros1_queue_size"), "set_ros1_queue_size", "get_ros1_queue_size");      \
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ros1_pub_rate"), "set_ros1_pub_rate", "get_ros1_pub_rate");          \
    ADD_PROPERTY(PropertyInfo(Variant::INT, "ros1_pub_type_rate"), "set_ros1_pub_type_rate", "get_ros1_pub_type_rate");
