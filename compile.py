import os
import argparse
import sys


def configure_and_build(config, target, num_cores, build_system, out_file, force_cmake):


    l_multi_config_build_substr = ""
    l_multi_config_cmake_config_str = ""
    if config == "Ninja Multi-Config":
        l_multi_config_build_substr= f" --config {config}"
    else:
        l_multi_config_cmake_config_str = f" -DCMAKE_BUILD_TYPE={config}"

    l_redirect_logging =""

    if out_file != "":
        l_redirect_logging = f" > {out_file} 2>&1"

        #Print where the output is redirected
        print(f"Redirecting output to file: {out_file}")


    l_conan_path = f"build/conan/{config}"
    l_cmake_path = f"build/cmake/{config}"

    #Chek if cmake dir has to be removed and created again
    if force_cmake == "y":
        print(f"Removing CMakeCache.txt file")
        os.system(f"rm -rf build/cmake/{l_cmake_path}/*")


    # CMake configure for Make or simple Ninja
    config_cmake_cmd = f". {l_conan_path}/conanbuild.sh && cmake . -G '{build_system}' -B {l_cmake_path} -DCMAKE_TOOLCHAIN_FILE={l_conan_path}/conan_toolchain.cmake {l_multi_config_cmake_config_str} {l_redirect_logging}  && cmake --version"

    print(f"Configuring CMake with command\n {config_cmake_cmd} \n")

    status = os.system(config_cmake_cmd)

    if status != 0:
        #Print error and exit
        print(f"Error configuring CMake with command\n {config_cmake_cmd} \n")
        sys.exit(status)

    # Build target
    status = os.system(f"chmod +x {l_conan_path}/conanbuild.sh")

    if status != 0:
        #Print error and exit
        sys.stderr.write(f"Error making conanbuild.sh executable with command\n chmod +x build/conanbuild.sh \n")
        sys.exit(status)

    #Print current working directory to check where compile_cmd is executed
    print(f"Current working directory: {os.getcwd()}")


    compile_cmd = f". {l_conan_path}/conanbuild.sh && cmake --version && cmake --build {l_cmake_path} --target {target} {l_multi_config_build_substr} -- -j{num_cores} {l_redirect_logging}"

    print(f"Compiling with command\n {compile_cmd} \n" )

    status = os.system(compile_cmd)

    if status != 0:
        #Print error and exit
        sys.stderr.write(f"Error compiling with command\n {compile_cmd}, check file with redirected output if it was provided \n")
        sys.exit(status)
        print("after exit")


if __name__ == "__main__":
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(description='Configure and build a CMake project.')

    # Add arguments for configuration
    parser.add_argument('--config', default="Release", help='CMake configuration (default: Release)')
    parser.add_argument('--target', required=True, default="all", help='Target to build (default: all)')
    parser.add_argument('--num-cores', type=int, default=1, help='Number of CPU cores to use for compilation (default: 1)')
    parser.add_argument('--build-system', default="Ninja", help='CMake build system generator (default: Ninja)')
    
    # Add argument for the output file
    parser.add_argument('--out-file', default="aux_gen/configure_and_compile_res.sh", help='Path to the output log file (default: aux_gen/configure_and_compile_res.sh)')

    parser.add_argument("--force-cmake", default="y", help="Force CMake to run even if the CMakeCache.txt file exists (default: y, values: y/n)") 

    # Parse the command-line arguments
    args = parser.parse_args()

    # Call the configure_and_build function with the parsed arguments
    configure_and_build(args.config, args.target, args.num_cores, args.build_system, args.out_file, args.force_cmake)
