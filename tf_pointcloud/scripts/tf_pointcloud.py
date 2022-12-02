#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_py as tf2

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# from transform_point_cloud.cfg import LookupTransformConfig

class TransformPointCloud:
    def __init__(self):
        
        rospy.loginfo("entering constructor")
        # params
        self.output_topic_name = rospy.get_param('~output_topic','velodyne_points_transformed')
        self.input_topic_name = rospy.get_param('~input_topic','velodyne_points')
        self.target_frame = rospy.get_param('~target_frame','velodyne')
        # self.source_frame = rospy.get_param('~source_frame','velodyne') # shouldn't need source frame
        # self.tf_timeout = rospy.get_param('~tf_timeout',2.0)
        self.tf_buffer_duration = rospy.get_param('~tf_buffer_duration',2)
        
        # transform listener
        # self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(self.tf_buffer_duration))
        self.tf_buffer = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        
        # publishers
        self.pub = rospy.Publisher(self.output_topic_name, PointCloud2, queue_size=2)
        
        # subscribers
        self.sub = rospy.Subscriber(self.input_topic_name, PointCloud2,self.point_cloud_callback, queue_size=5)
        
 
    def point_cloud_callback(self, msg):
        
        lookup_time = msg.header.stamp 
        target_frame = self.target_frame
        source_frame = msg.header.frame_id 
        try:
            # trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time)
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
        except tf2.LookupException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(msg, trans)
        cloud_out.header.stamp = msg.header.stamp
        self.pub.publish(cloud_out)
        

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud')
    transform_point_cloud = TransformPointCloud()
    rospy.spin()
