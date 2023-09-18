
import os

import argparse
import urdf_parser
from urdf_parser import parse_urdf



def main():
    parser = argparse.ArgumentParser(description="Parse URDF file")
    parser.add_argument("urdf_file", help="Path to the URDF file")
    parser.add_argument(
        "--transmission_files",
        nargs='+',
        help="List of transmission files"
    )

    args = parser.parse_args()
    urdf_file = args.urdf_file
    transmission_files = args.transmission_files
    
    l_parsed_urdf = parse_urdf(urdf_file, transmission_files)

if __name__ == "__main__":
    main()
