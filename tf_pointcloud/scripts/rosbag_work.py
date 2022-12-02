#!/usr/bin/env python

# fix for import error for 'import pcl' in python on ubuntu 18.04
# git clone https://github.com/wanguangxi/Python-PCL-Ubuntu18.04.git
# Installing collected packages: filelock, numpy, Cython, nose, six, funcsigs, mock, python-pcl
# Successfully installed Cython-0.29.32 filelock-3.2.1 funcsigs-1.0.2 mock-3.0.5 nose-1.3.7 numpy-1.16.6
# python-pcl-0.3.0rc1 six-1.16.0

import rosbag
import pcl
import rospy
import argparse
import sys
import subprocess, yaml



def main(args):
	
	parser = argparse.ArgumentParser(description='Concatenate point clouds of a bag file.')
	parser.add_argument('bagfile', nargs=1, help='input bag file')
	args = parser.parse_args()
	
	bagfile = args.bagfile[0]
	
	info_dict = yaml.load(rosbag.bag.Bag(bagfile, 'r')._get_yaml_info())
	
	for topic, msg, t in rosbag.Bag(bagfile).read_messages():
	
		print("hello")


if __name__ == "__main__":
	main(sys.argv[1:])
