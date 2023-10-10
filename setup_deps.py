
import os
import argparse


def gen_setup(f_args):

    config = f_args.config
    out_file = f_args.out_file

    l_build_option = "missing"

    if f_args.force_build == "y":
        l_build_option = '*'


    l_compiler = f_args.compiler
    conan_cmd = ""
    if l_compiler == "gcc":

        conan_cmd = f"conan install . --output-folder=build/conan/{config} --build={l_build_option} -pr:h conan/gcc_profile -pr:b conan/gcc_profile -s build_type={config} -s:b build_type={config}"
    elif l_compiler == "clang-12":

        conan_cmd = f"conan install . --output-folder=build/conan/{config} --build={l_build_option} -pr:h conan/clang12_profile -pr:b conan/clang12_profile -s build_type={config} -s:b build_type={config}"
    elif l_compiler == "clang-10":

        conan_cmd = f"conan install . --output-folder=build/conan/{config} --build={l_build_option} -pr:h conan/clang10_profile -pr:b conan/clang10_profile -s build_type={config} -s:b build_type={config}"
    else:
        raise Exception("Error, compiler not supported")


    if out_file != "":
        print(f"Redirecting output to file: {out_file}")
        conan_cmd += f" > {out_file} 2>&1"


    print(f"Configuring project with \n {conan_cmd} \n")

    status = os.system(conan_cmd)

    if status != 0:
        #Print error and exit
        print(f"Error configuring project with command\n {conan_cmd} \n")
        exit(status)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Generate all setup (dependencies, build tools, etc.)')

    # Add an argument for the 'out-file' parameter
    parser.add_argument('--out-file', default="aux_gen/setup_res.sh", help='Name of the file where to redirect the commands output instead of console. If emtpy string passed then output will be in the console (default: aux_gen/setup_res.sh)')
    parser.add_argument('--config', default="all", help='Configuration to compile deps (default: all, options: all, Debug, Release)')

    parser.add_argument("--force-build", default="n", help="Force build all packages. If not set only not conan prebuilt avaliables are built (default:n, options: y,n)")

    parser.add_argument("--compiler", default="gcc", help="Which compiler to use.  (default value gcc, options:gcc, clang-10, clang-12). gcc compiler is used with ld gnu linker, clang with lld one")

    # Parse the command-line arguments
    args = parser.parse_args()

    # Call the function with the specified arguments

    #Generate in function of the configuration

    if args.config == "all":
        args.config = "Debug"
        gen_setup(args)

        args.config = "Release"
        gen_setup(args)

    else:
        gen_setup(args)
