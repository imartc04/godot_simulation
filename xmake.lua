add_rules("mode.debug", "mode.release")

add_requires("protobuf-cpp")
add_requires("grpc 1.54.3" , {system = false})


local ros_lib_dirs = "/opt/ros/noetic/lib/"
local ros_include_dirs = "/opt/ros/noetic/include"

--Get protoc binary



-- print("exec dir ", l_exec)

-- os.execute("pwd")
--os.execute("rm -rf src/grpc_interface/gen_protoc/*")

local proto_files_path = "$(curdir)/src/grpc_interface/proto/"

target("ros1_pub_node")
    set_kind("binary")
    set_languages("c++17")
    add_files("src/ros1_nodes/ros1_pub_node.cpp")
    add_includedirs("src/")
    add_includedirs(ros_include_dirs)

    add_packages("protobuf-cpp")
    add_packages("grpc")

    add_rules("protobuf.cpp")

    -- add_files(proto_files_path .. "/*.proto" , {proto_rootdir = proto_files_path, proto_grpc_cpp_plugin = true})
    -- add_files(proto_files_path .. "/*.proto", {proto_rootdir = proto_files_path})

