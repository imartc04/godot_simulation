
import os
import argparse


def gen_setup(out_file, config):


    conan_cmd = f"conan install . --output-folder=build/conan/{config} --build=missing -s compiler.cppstd=17 -s compiler=gcc -s:b compiler.cppstd=17 -s:b compiler=gcc -s build_type={config} -s:b build_type={config}"

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
    parser.add_argument('--config', default="Release", help='Configuration to compile deps (default: Release)')

    # Parse the command-line arguments
    args = parser.parse_args()

    # Call the function with the specified arguments


    gen_setup(args.out_file, args.config)
