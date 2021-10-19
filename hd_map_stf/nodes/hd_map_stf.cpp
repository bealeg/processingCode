


#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <GeographicLib/Geocentric.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hd_map_stf");
  ros::NodeHandle node;
  ros::NodeHandle private_n("~");
  
  tf2_ros::StaticTransformBroadcaster stf_bc;
  
  GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
  
  std::string map_frame;
  double origin_lat;
  double origin_lon;
  double origin_h;
  
  private_n.getParam("map_frame", map_frame);
	private_n.getParam("map_origin_lat", origin_lat);
	private_n.getParam("map_origin_lon", origin_lon);
	private_n.getParam("map_origin_h", origin_h);
	ROS_INFO("map lat: %15.10f", origin_lat);
  ROS_INFO("map lon: %15.10f", origin_lon);
  ROS_INFO("map height: %15.10f", origin_h);
  
  std::vector<double> rotation_vec(9, 0.0);
  double x, y, z;
  earth.Forward(origin_lat, origin_lon, origin_h, x, y, z, rotation_vec);
  
  ROS_INFO("map x: %15.10f", x);
  ROS_INFO("map y: %15.10f", y);
  ROS_INFO("map z: %15.10f", z);
  
  tf2::Vector3 origin(x, y, z);
  tf2::Matrix3x3 rotation;
  
  rotation.setValue(
        rotation_vec[0],
        rotation_vec[1],
        rotation_vec[2],
        rotation_vec[3],
        rotation_vec[4],
        rotation_vec[5],
        rotation_vec[6],
        rotation_vec[7],
        rotation_vec[8]);
  
  tf2::Transform earth_hdmap_tf(rotation, origin);
  
  ros::Rate loop_rate(10);
  
  while (ros::ok()) {
		
		geometry_msgs::TransformStamped earth_hdmap_tfs_msg;
		earth_hdmap_tfs_msg.header.stamp = ros::Time::now();
		earth_hdmap_tfs_msg.header.frame_id = "earth";
		earth_hdmap_tfs_msg.child_frame_id = map_frame;
		tf2::convert(earth_hdmap_tf, earth_hdmap_tfs_msg.transform);
		stf_bc.sendTransform(earth_hdmap_tfs_msg);
	
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
    
    loop_rate.sleep();
  }
	
  return 0;

}
