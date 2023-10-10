#pragma once

#define GRPC_BIND_GODOT_METHDOS(CLASS)                                                                                        \
    ClassDB::bind_method(D_METHOD("set_ros1_grpc_server_ip", "ros1_grpc_server_ip"), &CLASS::set_ros1_grpc_server_ip);        \
    ClassDB::bind_method(D_METHOD("get_ros1_grpc_server_ip"), &CLASS::get_ros1_grpc_server_ip);                               \
    ClassDB::bind_method(D_METHOD("get_ros1_grpc_server_port"), &CLASS::get_ros1_grpc_server_port);                           \
    ClassDB::bind_method(D_METHOD("set_ros1_grpc_server_port", "ros1_grpc_server_port"), &CLASS::set_ros1_grpc_server_port);  \
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "ros1_grpc_server_ip"), "set_ros1_grpc_server_ip", "get_ros1_grpc_server_ip"); \
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "ros1_grpc_server_port"), "set_ros1_grpc_server_port", "get_ros1_grpc_server_port");

#define GRPC_DECLARE_METHODS                                   \
    void set_ros1_grpc_server_ip(::godot::String f_grpc_server_ip);     \
    ::godot::String get_ros1_grpc_server_ip();                          \
    void set_ros1_grpc_server_port(::godot::String f_grpc_server_port); \
    ::godot::String get_ros1_grpc_server_port();

#define GRPC_DEFINE_METHODS(CLASS, MEMBER_CONFIG)                                                                                 \
    void CLASS::set_ros1_grpc_server_ip(::godot::String f_grpc_server_ip)                                                                \
    {                                                                                                                           \
        MEMBER_CONFIG.server_address = f_grpc_server_ip.utf8().get_data(); \
    }                                                                                                                           \
    ::godot::String CLASS::get_ros1_grpc_server_ip()                                                                                     \
    {                                                                                                                           \
        return MEMBER_CONFIG.server_address.c_str();                       \
    }                                                                                                                           \
    void CLASS::set_ros1_grpc_server_port(String f_grpc_server_port)                                                            \
    {                                                                                                                           \
        MEMBER_CONFIG.server_port = f_grpc_server_port.utf8().get_data();  \
    }                                                                                                                           \
    String CLASS::get_ros1_grpc_server_port()                                                                                   \
    {                                                                                                                           \
        return MEMBER_CONFIG.server_port.c_str();                          \
    }