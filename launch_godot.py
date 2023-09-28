import os
import sys
import argparse

def launch_with_path(exec_path):

    # Append the specified path to the current PATH variable
    l_path_append = os.getcwd() + "/godot_proj/bin/gen/"

    # Launch the executable
    status = os.system(f"export GODOT_GEN_BIN_DIR={os.getcwd()}/godot_proj/bin/gen && export PATH=$PATH:{l_path_append} && {exec_path} godot_proj/project.godot")

    # Exit with the same status as the executable and print if some error occurred based on status
    if status != 0:
        print(f"Error launching executable with command:\n{exec_path}\n")

    sys.exit(status)

def main():
    parser = argparse.ArgumentParser(description="Launch an executable with an updated PATH variable.")

    l_def = "/root/Godot_v4.1.1-stable_linux.x86_64"
    parser.add_argument("--exec-path", default=l_def, help=f"Path to the executable to launch. (default: {l_def})")

    args = parser.parse_args()

    if not os.path.isfile(args.exec_path):
        print(f"Error: The specified executable '{args.exec_path}' does not exist.")
        sys.exit(1)

    launch_with_path(args.exec_path)

if __name__ == "__main__":
    main()
