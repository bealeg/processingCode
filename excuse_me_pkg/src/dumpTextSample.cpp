#include <vector>
#include <chrono>
#include <stdio.h>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <GeographicLib/Geocentric.hpp>

#include <tf/transform_listener.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

/* Gregory Beale
 * version 0.2 2021-05-13
 * ROS nodes that dump the information from a tracking module in 
 * autowar Kalman filter tracking module. Outputs timestamp, objectID, 
 * frameID, x y and z dimensions, x y z centroid position, yaw for 
 * heading, linear velocity in x y z, angular velocity in x y and z 
 * directions. As of now, the yaw provides heading and the linear speed
 * in x is the overall speed (and linear velocities in y and z are 0), 
 * as well as the angular velocities are always 0.
 * All outputs are written to a csv file.
 * */


/* callback for the objectArray message
 * Loops through each objectArray (autoware custom message) and outputs
 * the tracking information from each frame
 */
 
class dumpResultsNode
{
private:

  ros::NodeHandle n;
	
  ros::Subscriber obj_detected_sub;
  ros::Subscriber veh_pos_sub;
  ros::Subscriber veh_vel_sub;
  
  geometry_msgs::PoseStamped veh_pose;
  geometry_msgs::TwistStamped veh_velo;

  tf::TransformListener *tf_listener_ptr_;
  tf::StampedTransform local2global_;
  tf::StampedTransform local2llh_;
  tf::StampedTransform map2llh_;
  
  std::string result_file_path_;
  std::string result_file_path_ego;
  std::ofstream outputfile;
  
	void poseCB(const geometry_msgs::PoseStamped &curr_veh_pose){
		veh_pose = curr_veh_pose;
	}
	
	void velCB(const geometry_msgs::TwistStamped &curr_veh_velo){
		veh_velo = curr_veh_velo;
	}
	
	geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose,const tf::StampedTransform& tf_stamp)
	{
		tf::Transform transform;
		geometry_msgs::PoseStamped out_pose;
		transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
		transform.setRotation(
				tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
		geometry_msgs::PoseStamped pose_out;
		tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
		return out_pose.pose;
	}
	
	void transformPoseToGlobal(const autoware_msgs::DetectedObjectArray& input,
                                      autoware_msgs::DetectedObjectArray& transformed_input)
	{
		transformed_input.header.stamp = input.header.stamp;
		transformed_input.header.frame_id = "map";
		for (auto const &object: input.objects)
		{
			geometry_msgs::Pose out_pose = getTransformedPose(object.pose, local2global_);
			
			autoware_msgs::DetectedObject dd;
			dd = object;
			dd.header = transformed_input.header;
			dd.pose = out_pose;

			transformed_input.objects.push_back(dd);
		}
	}
	
	void detectedObjectCB(const autoware_msgs::DetectedObjectArray& detected_objects)
	{
		// look up the transform from base link to map, map to earth, and base_link_ground to earth
		try
		{
			//tf_listener_.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
			tf_listener_ptr_->lookupTransform("map", "base_link_ground", ros::Time(0), local2global_);
			tf_listener_ptr_->lookupTransform("earth", "map", ros::Time(0), map2llh_);
			tf_listener_ptr_->lookupTransform("earth", "base_link_ground", ros::Time(0), local2llh_);
			
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		//ROS_INFO("vehicle pose in ecef x: %f y: %f z: %f",veh_ecef_pose.position.x, veh_ecef_pose.position.y, veh_ecef_pose.position.z);
		
		autoware_msgs::DetectedObjectArray transformed_input;
		transformPoseToGlobal(detected_objects, transformed_input); //transform to map frame
		
		
		GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
		
		std::ofstream outputfile(result_file_path_, std::ofstream::out | std::ofstream::app);
		// loop through each object in the object array
		
		for (size_t i = 0; i < detected_objects.objects.size(); i++)
		{
			double yaw = tf::getYaw(detected_objects.objects[i].pose.orientation); // gets yaw fom quaternion
			
			//write information to csv
			// << std::to_string(detected_objects.objects[i].header.frame_id) << ","
			
			std::string originalStr = 
									std::to_string(detected_objects.header.stamp.toSec()) + "," 
								+ std::to_string(detected_objects.objects[i].id) + ","
								+ detected_objects.objects[i].header.frame_id + ","
								+ std::to_string(detected_objects.objects[i].dimensions.x) + ","
								+ std::to_string(detected_objects.objects[i].dimensions.y) + ","
								+ std::to_string(detected_objects.objects[i].dimensions.z) + ","
								+ std::to_string(detected_objects.objects[i].pose.position.x) + ","
								+ std::to_string(detected_objects.objects[i].pose.position.y) + ","
								+ std::to_string(detected_objects.objects[i].pose.position.z) + ","
								+ std::to_string(yaw) + ","
								+ std::to_string(detected_objects.objects[i].velocity.linear.x) + "," //apparently target velocity, not just in the x component
								+ std::to_string(detected_objects.objects[i].velocity.linear.y) + "," //apparently yaw rate!! need to be checked
								+ std::to_string(detected_objects.objects[i].velocity.linear.z) + ","
								+ std::to_string(detected_objects.objects[i].velocity.angular.x) + ","
								+ std::to_string(detected_objects.objects[i].velocity.angular.y) + ","
								+ std::to_string(detected_objects.objects[i].velocity.angular.z);
			
			double yawTF = tf::getYaw(transformed_input.objects[i].pose.orientation); // gets yaw fom quaternion
			std::string transformedStr = 
									std::to_string(transformed_input.header.stamp.toSec()) + ","
								+ transformed_input.objects[i].header.frame_id + ","
								+ std::to_string(transformed_input.objects[i].id) + "," 
								+ std::to_string(transformed_input.objects[i].dimensions.x) + ","
								+ std::to_string(transformed_input.objects[i].dimensions.y) + ","
								+ std::to_string(transformed_input.objects[i].dimensions.z) + ","
								+ std::to_string(transformed_input.objects[i].pose.position.x) + ","
								+ std::to_string(transformed_input.objects[i].pose.position.y) + ","
								+ std::to_string(transformed_input.objects[i].pose.position.z) + ","
								+ std::to_string(yawTF);
								
			geometry_msgs::Pose obj_ecef_pose = getTransformedPose(detected_objects.objects[i].pose, local2llh_);
			
			double obj_lat, obj_long, obj_height;
			earth.Reverse(obj_ecef_pose.position.x, obj_ecef_pose.position.y, obj_ecef_pose.position.z, obj_lat, obj_long, obj_height);
			
			std::string latlongStr = 
									std::to_string(detected_objects.objects[i].id)+ "," + std::to_string(obj_lat) + "," + std::to_string(obj_long) + "," + std::to_string(obj_height);
								
			outputfile << originalStr << "," << "," << transformedStr << "," << "," << latlongStr << "\n";
		}
		
		geometry_msgs::Pose veh_ecef_pose = getTransformedPose(veh_pose.pose, map2llh_);
		
    // Convert ECEF to LLA
    double v_lat, v_long, v_height;
    earth.Reverse(veh_ecef_pose.position.x, veh_ecef_pose.position.y, veh_ecef_pose.position.z, v_lat, v_long, v_height);
		
		std::ofstream outputfileEgoVehicle(result_file_path_ego, std::ofstream::out | std::ofstream::app);
		
		double yaw_veh = tf::getYaw(veh_pose.pose.orientation); // gets yaw fom quaternion
		
		std::string veh_info = 
									std::to_string(veh_pose.header.stamp.toSec()) + "," 
								+ veh_pose.header.frame_id + ","
								+ std::to_string(veh_pose.pose.position.x) + "," 
								+ std::to_string(veh_pose.pose.position.y) + ","
								+ std::to_string(veh_pose.pose.position.z) + ","
								+ std::to_string(v_lat) + ","
								+ std::to_string(v_long) + ","
								+ std::to_string(v_height) + ","
								+ std::to_string(yaw_veh) + ","
								+ std::to_string(veh_velo.twist.linear.x) + "," //overall velo
								+ std::to_string(veh_velo.twist.linear.y) + "," //should always be 0
								+ std::to_string(veh_velo.twist.linear.z) + ","
								+ std::to_string(veh_velo.twist.angular.x) + ","
								+ std::to_string(veh_velo.twist.angular.y) + ","
								+ std::to_string(veh_velo.twist.angular.z);
								
		outputfileEgoVehicle << veh_info << "\n";
		
		// set up output file and ofstream
		// std::string result_file_path_ = "/home/gbeale/autoware.ai/src/excuse_me_addon/excuse_me_pkg/trackingResult.csv";
		
		
	}
public:

  dumpResultsNode(tf::TransformListener* in_tf_listener_ptr):n("~")
  {
    tf_listener_ptr_ = in_tf_listener_ptr;
  }
  
  void Run()
  {
		
		result_file_path_ = "/home/gbeale/autoware.ai/src/processingCode/excuse_me_pkg/trackingResult.csv";
		result_file_path_ego = "/home/gbeale/autoware.ai/src/processingCode/excuse_me_pkg/trackingResultEgoVeh.csv";
		
		ROS_INFO("Ready");
		
		ros::Subscriber veh_vel_sub = n.subscribe("/current_pose", 1000, &dumpResultsNode::poseCB, this);	
		ros::Subscriber obj_detected_sub = n.subscribe("/current_velocity", 1000, &dumpResultsNode::velCB, this);
		ros::Subscriber veh_pos_sub = n.subscribe("/detection/object_tracker/objects", 1000, &dumpResultsNode::detectedObjectCB, this);		
		
		ros::spin();
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_info_recorder");
  tf::TransformListener tf_listener;
  dumpResultsNode app(&tf_listener);
  app.Run();

  return 0;

}
