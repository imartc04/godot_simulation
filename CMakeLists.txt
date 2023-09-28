# Set cmake minimum version
cmake_minimum_required(VERSION 3.27)

# Set project name
project(GodotSimulation)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/godot_proj/bin/gen)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/godot_proj/bin/gen)

find_package(protobuf REQUIRED)

# set(gRPC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build) find_package(Threads
# REQUIRED)

find_package(gRPC REQUIRED)

#Print gRPC_INCLUDE_DIRS
message(STATUS "*******************gRPC_INCLUDE_DIRS : " ${gRPC_INCLUDE_DIRS})



find_package(Boost COMPONENTS chrono timer system headers REQUIRED)

# Get ros libraries through glob on /opt/ros/noetic/lib
file(GLOB ros_libs /opt/ros/noetic/lib/*.a /opt/ros/noetic/lib/*.so)

set(ros_include_base /opt/ros/noetic/include)
file(
  GLOB ros_include_path
  LIST_DIRECTORIES true
  "${ros_include_base}" "${ros_include_base}/*")

message(STATUS "*******************ros_include_path : " ${ros_include_path})

# Get godot-cpp dependency

# Get ros1 node executable source files
set(ros1_node_src_file src/ros1_nodes/ros1_pub_node.cpp)

file(GLOB grpc_interface_src src/grpc_interface/gen_protoc/*.cpp
     src/grpc_interface/gen_protoc/*.cc)

# Pring grpc_interface_src
message(
  STATUS "\n********\ngrpc_interface_src: ${grpc_interface_src} \n ********")

# Get godot-cpp target
add_subdirectory(deps/godot-cpp)

# Set target for ros 1 node
add_executable(ros1_pub_node ${ros1_node_src_file} ${grpc_interface_src})

target_include_directories(ros1_pub_node PRIVATE ${CMAKE_SOURCE_DIR}/src
                                                 /opt/ros/noetic/include/


                                                 $<$<CONFIG:Debug>:${gRPC_INCLUDE_DIRS}>
                                                # /root/.conan2/p/b/proto2ad1b46c8da22/p/include/
                                                 )

target_link_libraries(ros1_pub_node PRIVATE ${ros_libs} gRPC::grpc++)

# Get GDExtension src files
file(
  GLOB_RECURSE
  gd_extension_src
  "src/godot_register_types/*.cpp"
  "src/grpc_interface/*.cpp"
  "src/grpc_interface/*.cc"
  "src/ros_interface/*.cpp"
  "src/virtual_sensors/*.cpp")

# Pring gd_extension_src message(STATUS "\n********\ngd_extension_src:
# ${gd_extension_src} \n ********")

add_library(godot_ros_extension SHARED ${gd_extension_src})

target_link_libraries(godot_ros_extension PRIVATE gRPC::grpc++ godot::cpp
                                                  ${ros_libs} Boost::headers )

target_include_directories(godot_ros_extension PRIVATE ${CMAKE_SOURCE_DIR}/src
                                                       /opt/ros/noetic/include/
                                                       )
# Create custom target to run custom command before build
add_custom_target(compile_gRPC)

# Set custom command to compile protoc files
add_custom_command(
  TARGET compile_gRPC
  PRE_BUILD
  WORKING_DIRECTORY $<TARGET_FILE_DIR:gRPC::grpc_cpp_plugin>
  COMMAND
    protobuf::protoc -I=${CMAKE_SOURCE_DIR}/src/grpc_interface/proto
    --cpp_out=${CMAKE_SOURCE_DIR}/src/grpc_interface/gen_protoc
    --grpc_out=${CMAKE_SOURCE_DIR}/src/grpc_interface/gen_protoc
    --plugin=protoc-gen-grpc=$<TARGET_FILE_NAME:gRPC::grpc_cpp_plugin>
    ${CMAKE_SOURCE_DIR}/src/grpc_interface/proto/*.proto
  COMMENT "Generating gRPC files")

add_dependencies(godot_ros_extension compile_gRPC)
add_dependencies(ros1_pub_node compile_gRPC)

# add_custom_target(build_all DEPENDS ros1_pub_node godot_ros_extension)

# Add test project

add_executable(test_main test_main.cpp)

target_link_libraries(test_main gRPC::grpc++ ${ros_libs})
add_dependencies(test_main compile_gRPC)
