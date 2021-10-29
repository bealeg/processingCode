


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
	// lat, lon, height values for each site location
  std::vector<double>   roanoke_1{37.28268805642265, -79.93856609724196, 265.88}; //roanoke llh
  std::vector<double>   roanoke_2{37.268866667532,   -79.9365594337049,  249};
  std::vector<double> charlotte_1{35.27768402841451, -80.81067547979933, 200}; // charrlotte llh
  std::vector<double> charlotte_2{35.27448813897137, -80.8299064623742, 200};
  std::vector<double>        dc_1{38.87684084686699, -76.97034236117101, -30}; // NEED TO FIX. orig. 38.87682084686699, -76.97034236117101
  std::vector<double>        dc_2{38.89844236697333, -76.94935333339094, -24}; // NEED TO FIX orig. 38.89844236697333, -76.94933333339094
  
  
  ros::init(argc, argv, "hd_map_stf");
  ros::NodeHandle node;
  ros::NodeHandle private_n("~");
  
  tf2_ros::StaticTransformBroadcaster stf_bc;
  
  GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
  
  std::string map_frame;
  std::string site_location;
  double origin_lat;
  double origin_lon;
  double origin_h;
  
  private_n.getParam("site_location",site_location);
  private_n.getParam("map_frame", map_frame);
  
  ROS_INFO("[HD_MAP_STF] site location: %s", site_location.c_str());
  
  if (site_location.find("roanoke_1") != std::string::npos) { // if roanoke 1
		origin_lat = roanoke_1[0];
		origin_lon = roanoke_1[1];
		origin_h = roanoke_1[2];
		ROS_INFO("[HD_MAP_STF] site location: ***roanoke_1***");
	} 
	else if (site_location.find("roanoke_2") != std::string::npos) { // if roanoke 2
		origin_lat = roanoke_2[0];
		origin_lon = roanoke_2[1];
		origin_h = roanoke_2[2];
		ROS_INFO("[HD_MAP_STF] site location: ***roanoke_2***");
	} 
	else if (site_location.find("charlotte_1") != std::string::npos) { // if charlotte 1
		origin_lat = charlotte_1[0];
		origin_lon = charlotte_1[1];
		origin_h = charlotte_1[2];
		ROS_INFO("[HD_MAP_STF] site location: ***charlotte_1***");
	} 
	else if (site_location.find("charlotte_2") != std::string::npos) { // if chartlotte 2
		origin_lat = charlotte_2[0];
		origin_lon = charlotte_2[1];
		origin_h = charlotte_2[2];
		ROS_INFO("[HD_MAP_STF] site location: ***charlotte_2***");
	} 
	else if (site_location.find("dc_1") != std::string::npos) { // if dc 1
		origin_lat = dc_1[0];
		origin_lon = dc_1[1];
		origin_h = dc_1[2];
		ROS_INFO("[HD_MAP_STF] site location: ***dc_1***");
	} 
	else if (site_location.find("dc_2") != std::string::npos) { // if dc 2
		origin_lat = dc_2[0];
		origin_lon = dc_2[1];
		origin_h = dc_2[2];
		ROS_INFO("\n[HD_MAP_STF] site location: ***dc_2***");
	}
	else { ROS_INFO("[HD_MAP_STF] site location not found!!"); }
  
	//private_n.getParam("map_origin_lat", origin_lat);
	//private_n.getParam("map_origin_lon", origin_lon);
	//private_n.getParam("map_origin_h", origin_h);
	ROS_INFO("map lat: %15.10f", origin_lat);
  ROS_INFO("map lon: %15.10f", origin_lon);
  ROS_INFO("map height: %15.10f", origin_h);
  
  std::vector<double> rotation_vec(9, 0.0);
  double x, y, z;
  earth.Forward(origin_lat, origin_lon, origin_h, x, y, z, rotation_vec);
  
  ROS_INFO("map x: %15.10f", x);
  ROS_INFO("map y: %15.10f", y);
  ROS_INFO("map z: %15.10f\n", z);
  
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
