import os
import sys
import scons_compiledb
import re


from pathlib import Path


"""
make new find_files function that will return a dictionary with different elements
depending on parameters passed.
The parameters will be:
 - root_folder: the root folder to start searching
 - extensions: the extensions to search for
 - names_with_full_paths : if true, returns a list with full paths
 - name_with_extensions: if true, the names will be returned with extensions
 - exclude_folders : list of folders to exclude from search when encountered

returns adictionary with the next elements
 - file_names: list of file names (with extensions if name_with_extensions is true and with full paths if names_with_full_paths is true)
 - paths : list of paths where the files were found
"""
def find_files(root_folder: str, extensions: list, names_with_full_paths: bool = False, names_with_extensions: bool = False, exclude_folders: list = []):
    if not root_folder.endswith("/"):
        root_folder += "/"
    file_names = []
    paths = []
    for root, dirs, files in os.walk(root_folder):
        for file in files:
            if file.endswith(tuple(extensions)):

                #I think is wrong here , one can have file with full path and without extension
                #Generate file names posibilities
                file_name = file
                if names_with_extensions:
                    file_name = file
                else:
                    file_name = os.path.splitext(file)[0]

                if names_with_full_paths:
                    file_name = root + "/" + file_name

                file_names.append(file_name)
               
                paths.append(root)
                
        for folder in exclude_folders:
            if folder in dirs:
                dirs.remove(folder)
    return {"file_names": file_names, "paths": paths}



original_cwd = os.getcwd()
print("original_cwd: ", original_cwd)

"""
Remember that the path of this file will be inisde of build, check the original_cwd message
if build variant dir is used with Scons. Scons will copy this
file to build
"""
l_repo_base_path = "./../"
print("l_repo_base_path: ", l_repo_base_path)



#os.chdir(l_repo_base_path + "/src/deps/godot-cpp")
env = SConscript(l_repo_base_path +"/deps/godot-cpp/SConstruct")

#os.chdir(original_cwd)



#env = Environment()

scons_compiledb.enable(env)

use_gcc = ARGUMENTS.get('use_gcc')

#Convert string to bool
if use_gcc == "True":
    use_gcc = True
elif use_gcc == "False":
    use_gcc = False

if use_gcc == None:
    use_gcc = False

print("use_gcc, " , use_gcc, " type(use_gcc) " , type(use_gcc))

# Enable debug symbols and disable optimization
if use_gcc:
    print("Configuring for gcc")
    env.Replace(CCFLAGS = [], CXXFLAGS = ["-g", "-O0", "-fPIC"], CPPDEFINES=['DEBUG',"DEBUG_ENABLED"])
    env.Replace(LINKFLAGS=["-g", "-O0", "-fPIC"])

    env.Append(LIBPATH=["/usr/local/lib/", "/usr/lib/x86_64-linux-gnu/", "/usr/lib/gcc/x86_64-linux-gnu/9/"])

else:
    print("Configuring for clang")

    env.Replace(CCFLAGS = [], CXXFLAGS = ["-g", "-O0", "-fPIC", "-stdlib=libc++" ])
    
    #-lc++ : Links llcm libc++ standard library
    #-fuse-ld=lld : Use llvm lld linker 
    env.Replace(LINKFLAGS=["-lc++", "-fuse-ld=lld"], CPPDEFINES=[])

    env.Append(LIBPATH=["/usr/lib/llvm-10/lib/"])
    env.Append(LIBPATH=["/usr/lib/x86_64-linux-gnu/"])
 

    #Set llvm clang compiler to use
    env['CXX'] = 'clang++'


#Set commmon paths
env.Append(CPPPATH=["/usr/include/"])

"""
Set common libs
rt - library for boost interprocess shared memory
"""
env.Append(LIBS=["pthread", "rt"])

ros_distro = "noetic"
cpp_version = "-std=c++17"

ros_dir = "/opt/ros/" + ros_distro

def getSubDirs(base_path: str, with_base_path: bool = True):
    if not base_path.endswith("/"):
        base_path += "/"
    sub_dirs = [name for name in os.listdir(base_path) if os.path.isdir(base_path + name)]
    if with_base_path:
        sub_dirs = [f"{base_path}/{name}" for name in sub_dirs]
    return sub_dirs


print("Adding ROS includes ...")

ros_includes = getSubDirs(ros_dir + "/include")

print("Done")

#print("\n ***********************************\n Type Ros include paths" , type(ros_includes), " *********\n")
# print("\n ***********************************\n Ros include paths" , ros_includes, " *********\n")

ros_lib_path = ros_dir + "/lib/"

# print("ros_lib_path: ", ros_lib_path)

# Add all cpp files to the build
# env.add_source_files(env.modules_sources, "src/demos/*.cpp")
# env.add_source_files(env.modules_sources, "*.cpp")

# env.Append(CPPPATH=ros_includes)


# These paths are relative to /modules/ros if they are not absolute
env.Append(CPPPATH=[ros_dir + "/include"])


env.Append(LIBPATH=[ros_lib_path])

# ROS needs c++ version compilier flag
env.Append(CCFLAGS=[cpp_version])

# Check with the documentation of the external library to see which library
# files should be included/linked.
ros_libs = find_files(root_folder=ros_lib_path, extensions= [".so", ".a"], names_with_full_paths=False, names_with_extensions=False, exclude_folders=["dist-packages"])

print("ros_libs: ", ros_libs)

#Add ros lib paths to linker rpath

for path in ros_libs["paths"]:
    #env.Append(RPATH= [path], LIBPATH= [path])
    #env.Append(LIBPATH= [path])
    pass

env.Append(LIBS= ros_libs["file_names"] )

#Add Boost libraries
#env.Append(LIBS=["libboost_system.so" ]) #, "libboost_filesystem.so"])



l_src_path = l_repo_base_path + "src/"

env.Append(CPPPATH=[".", l_src_path])

l_all_src_files = find_files(l_src_path, [".cpp"], True, True )["file_names"]

# for file in l_all_src_files:
#     print("src file :" , file)

# Define the regular expression pattern you want to match
pattern = ".*ros1_pub_node.cpp"  # For example, we'll remove strings containing "an"


# Use a list comprehension to filter out strings that match the pattern
l_gdextension_src_files = [item for item in l_all_src_files if not re.search(pattern, item)]


#Print all files
# for file in l_all_src_files:
#     print("src file :" , file)



ros1_pub_node = env.Program(
    target=  l_repo_base_path + "/godot_proj/bin/ros1_pub_node",
    source= l_repo_base_path + "/src/ros_interface/ros1/ros1_pub_node.cpp"
 
)   

sh_lib_dg_extension = env.SharedLibrary(
    target=  l_repo_base_path + "/godot_proj/bin/godot_ros_extension{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
    source= l_gdextension_src_files
)



# Add all targets to the Default function
Default(ros1_pub_node, sh_lib_dg_extension)

#Compile database for VScode
env.CompileDb()
