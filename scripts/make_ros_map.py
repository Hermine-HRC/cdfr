#!/usr/bin/env python3

"""
This script convert a binarized png to a ROS map.
Usage:
python3 make_ros_map
python3 make_ros_map path/to/my/image.png
"""

import cv2
import os.path
import sys

#
#  This is a start for the map program
#
prompt = '> '
resolution = 0.01

if len(sys.argv) != 2:
    print("What is the name of your floor plan you want to convert to a ROS map:")
    file_name = input(prompt)
else:
    file_name = sys.argv[1]

#
# Read in the image
#
image = cv2.imread(file_name)

# Convert to grey
res = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

map_name = file_name.split('/')[-1].split('.')[0]
map_location = '/'.join(file_name.split('/')[:-1])
complete_file_name_map = os.path.join(map_location, map_name + ".pgm")
complete_file_name_yaml = os.path.join(map_location, map_name + ".yaml")
with open(complete_file_name_yaml, "w") as yaml:
    cv2.imwrite(complete_file_name_map, res)
    # Write some information into the file
    yaml.write(f"image: {map_name}.pgm\n")
    yaml.write(f"resolution: {resolution}\n")
    yaml.write(f"origin: [{-image.shape[1] / 2 * resolution:.3f}, {-image.shape[0] / 2 * resolution:.3f}, 0]\n")
    yaml.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
