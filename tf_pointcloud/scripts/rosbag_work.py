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
import tf
import tf2_ros
import tf2_py as tf2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from inspect import getmembers, isfunction

"""
class RosBagWork():
	def __init__(self):
		
		rospy.loginfo("entering constructor")
		
		# params
		self.input_bag_file = rospy.get_param('~input_bag_file','/home/gbeale/Downloads/Bags/BoltOn/ShortBag.bag')
		self.output_bag_file = rospy.get_param('~output_bag_file','/home/gbeale/Downloads/Bags/BoltOn/')
		
		# get bag information
		self.bag = rosbag.Bag(self.input_bag_file)
		self.info_dict = yaml.load(rosbag.bag.Bag(self.input_bag_file, 'r')._get_yaml_info())
		# ['end', 'compression', 'types', 'topics', 'messages', 'uncompressed', 'start', 'version', 'compressed', 'indexed', 'path', 'duration', 'size']
		self.start_time = self.info_dict['start']
		self.end_time = self.info_dict['end']
		
		rospy.loginfo("Start time: %d", self.start_time)
		rospy.loginfo("End time: %d", self.end_time)
		rospy.loginfo("Duration: %d", self.end_time - self.start_time)
		rospy.loginfo(self.info_dict['duration'])
		rospy.loginfo(self.info_dict['size'])

		# transform listener
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(self.end_time - self.start_time))
		# self.tf_buffer = tf2_ros.Buffer()
		# self.tl = tf2_ros.TransformListener(self.tf_buffer)
		
	
	def run(self):
		
		rospy.loginfo("reading tf messages")
		for topic, msg, t in self.bag.read_messages(topics=['/tf','/tf_static']):
			if topic == "/tf" or topic == '/tf_static' and msg.transforms:
				for msg_tf in msg.transforms:
					self.tf_buffer.setTransform(msg_tf)
					# tf_t.setTransform(msg_tf)
					# t2_buff.setTransform(msg_tf)
		rospy.loginfo("finished loading tf messages")
		self.bag.close()
		rospy.signal_shutdown("Finished bag work")
"""

class RosBagWork():
	def __init__(self):
		
		print(dir(pcl.PointCloud))
		
		# params
		self.input_bag_file = rospy.get_param('~input_bag_file','/home/gbeale/Downloads/Bags/BoltOn/ShortBag2.bag')
		self.output_bag_file = rospy.get_param('~output_bag_file','/home/gbeale/Downloads/Bags/BoltOn/output.bag')
		self.write_bool = rospy.get_param('~write_bool',False)
		self.target_frame = rospy.get_param('~target_frame',"base_link_ground")
		self.t=None
		self.bag_in = rosbag.Bag(self.input_bag_file)
		
		print(self.write_bool)
		if self.write_bool==True:
			print("Writing outbag to file")
		else:
			print("Not writing result to file")
		
		self.info_dict = yaml.load(self.bag_in._get_yaml_info())
		# ['end', 'compression', 'types', 'topics', 'messages', 'uncompressed', 'start', 'version', 'compressed', 'indexed', 'path', 'duration', 'size']
		self.start_time = self.info_dict['start']
		self.end_time = self.info_dict['end']
		self.duration = self.info_dict['duration']
		self.compression_type = self.info_dict['compression']
		print("Start time: ", self.start_time)
		print("End time: ", self.end_time)
		print("Duration: ", self.duration)
		print("Compression type: ", self.compression_type)
		print(self.info_dict['size'])
		
		self.tf2_buff = tf2_ros.Buffer(rospy.Duration(self.end_time - self.start_time))
		# print(dir(tf2_buff))
		self.tf_t = tf.Transformer(True, rospy.Duration(self.end_time - self.start_time))
		# tf2_t = tf2_ros.Transformer(True, rospy.Duration())
		
		self.r_sub = Subscriber("/lidar_front_right/velodyne_points",PointCloud2)
		self.l_sub = Subscriber("/lidar_front_left/velodyne_points",PointCloud2)
		self.ats = ApproximateTimeSynchronizer([self.r_sub, self.l_sub], queue_size=5, slop=0.1)
		self.ats.registerCallback(self.concatenateClouds)
		
		if self.write_bool==True:
			# outbag = rosbag.Bag(result, 'w', compression='bz2', chunk_threshold=1048576)
			self.outbag = rosbag.Bag(self.output_bag_file, 'w', compression='bz2')
		
		
	def run(self):
		
		print("Reading tf messages")
		for topic, msg, t in self.bag_in.read_messages(topics=['/tf','/tf_static']):
			if topic == "/tf" or topic == '/tf_static' and msg.transforms:
				for msg_tf in msg.transforms:
					# tf_t.setTransform(msg_tf)
					self.tf2_buff.set_transform(msg_tf, "bag_reader") # "bag_reader" is a made up name for 'authority' reasons
			
			# if write_bool:
				# outbag.write(topic, msg, t)
		print("Finished loading tf messages")
		
		print("Reading messages and doing work")
		prev_step = False
		# front left and right lidars
		for topic, msg, t in self.bag_in.read_messages(topics= \
			['/lidar_front_left/velodyne_points','/lidar_front_right/velodyne_points', \
			'/lidar_rear/velodyne_points','/now_time','/tf','/tf_static']):
			
			# self.topic = topic
			# self.msg = msg
			self.t = t
			
			# print(dir(self.l_sub))
			# print("Found front r/l or rear point cloud")
			if topic == "/lidar_front_left/velodyne_points":
				# left_cloud = instantiate(msg)
				self.l_sub.signalMessage(msg)
			elif topic == "/lidar_front_right/velodyne_points":
				# right_cloud = instantiate(msg)
				self.r_sub.signalMessage(msg)
			elif topic == "/lidar_rear/velodyne_points":
				trans_cloud = self.transCloud(msg)
				self.write_msg(topic,msg,t)
			else:
				self.write_msg(topic,msg,t)
			
			
			progress = (t.to_sec() - self.start_time) / self.duration * 100
			step = (progress % 10) // 1
			
			if step < 0.5:
				it = True
				if prev_step == False:
					print('%f%%' % progress)
				prev_step = True
			else:
				prev_step = False

		print("Finished reading messages and doing work")
		
		
		self.bag_in.close()
		if self.write_bool==True:
			self.outbag.close()
			print(self.outbag.get_compression_info())
		
		rospy.signal_shutdown("Finished bag work")
	
	
	def write_msg(self, topic_in, msg_in, t_in):
		
		if self.write_bool==True:
			self.outbag.write(topic_in, msg_in, t_in)
	
	
	def transCloud(self, msg):
		
		lookup_time = msg.header.stamp 
		source_frame = msg.header.frame_id
		
		try:
			# trans = self.tf_buffer.lookup_transform_(target_frame, source_frame, lookup_time)
			trans = self.tf2_buff.lookup_transform_full(self.target_frame, rospy.Time(0), source_frame, rospy.Time(0),fixed_frame="base_link")
			# trans = tf_t.lookupTransform(target_frame, rospy.Time(0), source_frame)
		except Exception as ex:
			print(str(lookup_time.to_sec()))
			print(ex)
		
		cloud_out = do_transform_cloud(msg, trans)
		cloud_out.header.stamp = msg.header.stamp
		return cloud_out
	
	
	def concatenateClouds(self, rightCloud, leftCloud):# righCloud,leftCloud):
		
		print("got to the callback")
		right_tr_cloud = self.transCloud(rightCloud)
		left_tr_cloud = self.transCloud(leftCloud)
		concat_topic = "/concatenate_data/output"
		
		concat_cloud = pcl.concatenate(right_tr_cloud,left_tr_cloud)
		# self.write_msg(concat_topic,concat_cloud,self.t)
		
	"""
	def main(args):
	
	parser = argparse.ArgumentParser(description='Concatenate point clouds of a bag file.')
	parser.add_argument('bagfile', nargs=1, help='input bag file')
	parser.add_argument('write', nargs=1, help='bool flag to write output')
	args = parser.parse_args()
	
	bagfile = args.bagfile[0]
	bag_in = rosbag.Bag(bagfile)
	print(bag_in.get_compression_info())
	
	write_bool_in = args.write[0]
	if write_bool_in == "True":
		write_bool = True
	else: 
		write_bool = False
	print(write_bool)
	if write_bool==True:
		print("Writing outbag to file")
	else:
		print("Not writing result to file")
	
	info_dict = yaml.load(bag_in._get_yaml_info())
	# ['end', 'compression', 'types', 'topics', 'messages', 'uncompressed', 'start', 'version', 'compressed', 'indexed', 'path', 'duration', 'size']
	
	start_time = info_dict['start']
	end_time = info_dict['end']
	duration = info_dict['duration']
	compression_type = info_dict['compression']
	print("Start time: ", start_time)
	print("End time: ", end_time)
	print("Duration: ", duration)
	print("Compression type: ", compression_type)
	print(info_dict['duration'])
	print(info_dict['size'])
	
	topics = bag_in.get_type_and_topic_info()[1].keys()
	# print(topics)
	
	tf2_buff = tf2_ros.Buffer(rospy.Duration(end_time - start_time))
	# print(dir(tf2_buff))
	tf_t = tf.Transformer(True, rospy.Duration(end_time - start_time))
	# tf2_t = tf2_ros.Transformer(True, rospy.Duration())
	
	target_frame = "base_link_ground"
	r_sub = Subscriber("/lidar_front_right/velodyne_points",PointCloud2)
	l_sub = Subscriber("/lidar_front_left/velodyne_points",PointCloud2)
	ats = ApproximateTimeSynchronizer([r_sub, l_sub], queue_size=5, slop=0.1)
	ats.registerCallback(concatenateClouds)
	
	if write_bool==True:
		result = "/home/gbeale/Downloads/Bags/BoltOn/outbag.bag"
		outbag = rosbag.Bag(result, 'w', compression='bz2', chunk_threshold=1048576)
	
	
	print("Reading tf messages")
	for topic, msg, t in bag_in.read_messages(topics=['/tf','/tf_static']):
		if topic == "/tf" or topic == '/tf_static' and msg.transforms:
			for msg_tf in msg.transforms:
				# tf_t.setTransform(msg_tf)
				tf2_buff.set_transform(msg_tf, "bag_reader") # "bag_reader" is a made up name for 'authority' reasons
		
		# if write_bool:
			# outbag.write(topic, msg, t)
	print("Finished loading tf messages")
	
	
	print("Reading messages and doing work")
	prev_step = False
	# front left and right lidars
	for topic, msg, t in rosbag.Bag(bagfile).read_messages(topics= \
		['/lidar_front_left/velodyne_points','/lidar_front_right/velodyne_points', \
		'/lidar_rear/velodyne_points','/now_time','/tf','/tf_static']):
		
		if topic == "/lidar_front_left/velodyne_points" or topic == "/lidar_front_right/velodyne_points" or topic == "/lidar_rear/velodyne_points":
			
			print(dir(l_sub))
			# print("Found front r/l or rear point cloud")
			if topic == "/lidar_front_left/velodyne_points":
				# left_cloud = instantiate(msg)
				l_sub.signalMessage(msg)
			elif topic == "/lidar_front_right/velodyne_points":
				# right_cloud = instantiate(msg)
				r_sub.signalMessage(msg)
			
			lookup_time = msg.header.stamp 
			source_frame = msg.header.frame_id 
			
			# since front left/right lidars only need static transforms (which don't change, can just use most recent time
			# maybe this works?
			try:
				# trans = self.tf_buffer.lookup_transform_(target_frame, source_frame, lookup_time)
				trans = tf2_buff.lookup_transform_full(target_frame, rospy.Time(0), source_frame, rospy.Time(0),fixed_frame="base_link")
				# trans = tf_t.lookupTransform(target_frame, rospy.Time(0), source_frame)
			except Exception as ex:
				print(str(lookup_time.to_sec()))
				print(ex)
			
			cloud_out = do_transform_cloud(msg, trans)
			cloud_out.header.stamp = msg.header.stamp
			# print("Transformed %s topic to /base_link_ground frame",source_frame)
			
			if write_bool==True:
				outbag.write(topic + "_transformed", cloud_out, t)
		elif topic == "/tf" or topic == "/tf_static":
			if write_bool==True:
				outbag.write(topic, msg, t)
		
		
		progress = (t.to_sec() - start_time) / duration * 100
		step = (progress % 10) // 1
		
		if step < 0.5:
			it = True
			if prev_step == False:
				print('%f%%' % progress)
			prev_step = True
		else:
			prev_step = False

	print("Finished reading messages and doing work")
	bag_in.close()
	
	if write_bool==True:
		outbag.close()
		print(outbag.get_compression_info())
	"""



if __name__ == '__main__':
	try:
		rospy.init_node('rosbag_work',disable_signals=True)
		rosbag_work = RosBagWork()
		rosbag_work.run()
		# rospy.spin()
	except Exception as ex:
		print(ex)

		
		
# if __name__ == "__main__":
	# main(sys.argv[1:])
