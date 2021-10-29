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

 
class dumpResultsNode
{
private:

  ros::NodeHandle n;
  ros::NodeHandle private_n;
	
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
  std::string result_file_path_objs;
  std::string result_file_path_ego;
  std::string ego_header;
  std::string objs_header;
  
  std::string fixed_frame;
  std::string earth_frame;
  std::string objects_topic;
  
  std::ofstream outputfile;
  
  std::vector<std::vector<double> >  prev_id_list; // going to store time, id, x,y,z
  
  /*
	void poseCB(const geometry_msgs::PoseStamped &curr_veh_pose){
		veh_pose = curr_veh_pose;
	}
	*/
	
	// cb to store curent vehicle velocity
	void velCB(const geometry_msgs::TwistStamped &curr_veh_velo){
		veh_velo = curr_veh_velo;
	}
	
	// transform pose based on given tf
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
	
	// transform detected object array into global fixed frame
	void transformPoseToGlobal(const autoware_msgs::DetectedObjectArray& input,
                                      autoware_msgs::DetectedObjectArray& transformed_input)
	{
		transformed_input.header.stamp = input.header.stamp;
		transformed_input.header.frame_id = fixed_frame; // check fixed frame variable
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
	
	// cb to write detected object information to its own csv file
	void detectedObjectCB(const autoware_msgs::DetectedObjectArray& detected_objects)
	{
		// look up the transform from base_link_ground to map_veh, map_veh to earth, and base_link_ground to earth
		try
		{
			//tf_listener_.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
			tf_listener_ptr_->lookupTransform(fixed_frame, "base_link_ground", ros::Time(0), local2global_);
			tf_listener_ptr_->lookupTransform(earth_frame, fixed_frame, ros::Time(0), map2llh_);
			tf_listener_ptr_->lookupTransform(earth_frame, "base_link_ground", ros::Time(0), local2llh_);
			
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		//ROS_INFO("vehicle pose in ecef x: %f y: %f z: %f",veh_ecef_pose.position.x, veh_ecef_pose.position.y, veh_ecef_pose.position.z);
		
		//transform detected ibjects from tracker to map frame
		autoware_msgs::DetectedObjectArray transformed_input;
		transformPoseToGlobal(detected_objects, transformed_input); 
		
		//instanitate the geocentric object from geographic library
		GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
		
		//set up file to write to
		std::ofstream outputfile(result_file_path_objs, std::ofstream::out | std::ios::app);
		
		// loop through each object in the object array
		for (size_t i = 0; i < detected_objects.objects.size(); i++)
		{
			bool found = false;
			
			double yaw = tf::getYaw(detected_objects.objects[i].pose.orientation); // gets yaw fom quaternion
			
			// << std::to_string(detected_objects.objects[i].header.frame_id) << ","
			// add information to string for detected objects info in the base_link_ground frame
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
			
			// add information to string for detected objects info in the map_veh frame
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
			/*
			double velo;
			
			// loop through the previous ids and positions to calc a velo
			for (size_t j = 0; i < prev_id_list.size(); i++) {
				
				if (prev_id_list[j][1] == transformed_input.objects[i].id) {
					
					found = true;
					
					double curr_x = transformed_input.objects[i].pose.position.x;
					double curr_y = transformed_input.objects[i].pose.position.y;
					double curr_z = transformed_input.objects[i].pose.position.z;
					
					double prev_x = prev_id_list[j][2];
					double prev_y = prev_id_list[j][3];
					double prev_z = prev_id_list[j][4];
					
					double dt = transformed_input.header.stamp.toSec() - prev_id_list[j][0];
					
					double pos_change = sqrt(pow(curr_x - prev_x, 2) + pow(curr_y - prev_y, 2) + pow(curr_z - prev_z, 2));
					
					velo = pos_change / dt;
					
					//ROS_INFO("[DUMP TEXT] time diff for obj id %d:        %f", transformed_input.objects[i].id,dt);
					//ROS_INFO("[DUMP TEXT] position diff for id obj id %d: %f", transformed_input.objects[i].id,pos_change);
					//ROS_INFO("[DUMP TEXT] velo est for id obj id %d:      %f", transformed_input.objects[i].id,velo);
					break;
				}
			}
			
			if (found) {
				transformedStr = transformedStr + "," + std::to_string(velo);
			} else {
				//ROS_INFO("[DUMP TEXT] no match for obj id %d", transformed_input.objects[i].id);
				transformedStr = transformedStr + "," + "nan";
			}*/
			
			// transform object pose to ecef
			geometry_msgs::Pose obj_ecef_pose = getTransformedPose(detected_objects.objects[i].pose, local2llh_);
			// transform object pose from ecef to lat,lon,height
			double obj_lat, obj_long, obj_height;
			earth.Reverse(obj_ecef_pose.position.x, obj_ecef_pose.position.y, obj_ecef_pose.position.z, obj_lat, obj_long, obj_height);
			// write to string for lat lon height
			std::string latlongStr = 
									std::to_string(detected_objects.objects[i].id)+ "," + std::to_string(obj_lat) + "," + std::to_string(obj_long) + "," + std::to_string(obj_height);
			
			// write the strings to the file for the detected objects		
			outputfile << originalStr << "," << "," << transformedStr << "," << "," << latlongStr << "\n";
			
		}
		
		// clear and write the previous time, id, and position values to the id list
		/*
		prev_id_list.clear();
		//ROS_INFO("[DUMP TEXT] prev list size %d", prev_id_list.size());
		for (size_t j = 0; j < transformed_input.objects.size(); j++) {
			
			std::vector<double> input = { transformed_input.header.stamp.toSec(), static_cast<double> (transformed_input.objects[j].id) ,transformed_input.objects[j].pose.position.x,
				transformed_input.objects[j].pose.position.y, transformed_input.objects[j].pose.position.z};
			
			prev_id_list.push_back(input);
			
		}*/
		//ROS_INFO("[DUMP TEXT] prev list size %d", prev_id_list.size());
	}
	
	// cb to write the ego vehicle pose to a csv
	void poseCB(const geometry_msgs::PoseStamped &curr_veh_pose){
		
		veh_pose = curr_veh_pose;
		
		try
		{
			//tf_listener_.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
			tf_listener_ptr_->lookupTransform(earth_frame, fixed_frame, ros::Time(0), map2llh_); // check fixed_frame map variable
			
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		
		//transform vehicle pose to global fixed frame
		geometry_msgs::Pose veh_ecef_pose = getTransformedPose(veh_pose.pose, map2llh_);
		// instantiate geocentric object from geographic library
		GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
		
    // Convert ego vehicle positio to ECEF to lat,lon,height
    double v_lat, v_long, v_height;
    earth.Reverse(veh_ecef_pose.position.x, veh_ecef_pose.position.y, veh_ecef_pose.position.z, v_lat, v_long, v_height);
		// set up ofstream to write to file
		std::ofstream outputfileEgoVehicle(result_file_path_ego, std::ofstream::out | std::ios::app);
		
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
		
		// write vehicle information in map_veh frame and lat,lon,height information to csv		
		outputfileEgoVehicle << veh_info << "\n";
		
	}
	
public:

  dumpResultsNode(tf::TransformListener* in_tf_listener_ptr):n("~"), private_n("~")
  {
    tf_listener_ptr_ = in_tf_listener_ptr; // constructor takes a tf_listener as arg
  }
  
  void Run()
  {
		// read in params for the save path, the fixed_frame name, earth frame name, and object topic (can choose from filtered or non filtered)
		if (!private_n.getParam("save_path", result_file_path_)) {
			result_file_path_ = "/home/gbeale/autoware.ai/src/processingCode/excuse_me_pkg/";
		}
		if (!private_n.getParam("fixed_frame", fixed_frame)) {
			fixed_frame = "map_veh"; // using default as map_veh since we will leave all the vector map stuff as "map" frame
		}
		if (!private_n.getParam("earth_frame", earth_frame)) {
			earth_frame = "earth"; // using default as map_veh since we will leave all the vector map stuff as "map" frame
		}
		if (!private_n.getParam("objects_topic", objects_topic)) {
			objects_topic = "/detection/object_tracker/objects_filtered"; // using filtered objects as default
		}
		
		// create file paths from input
		result_file_path_objs = result_file_path_ + "_trackingResult.csv";
		result_file_path_ego = result_file_path_ + "_trackingResultEgoVeh.csv";
		
		
		std::ofstream outputfileEgoVehicle(result_file_path_ego, std::ofstream::out | std::ofstream::app);
		std::string ego_header = "timestamp,frame_id,x_pos,y_pos,z_pos,lat,lon,alt,yaw,vx,vy,vz,x_ang,y_ang,z_ang";
		outputfileEgoVehicle << ego_header << std::endl;
		outputfileEgoVehicle.close();
		
		std::ofstream outputfile(result_file_path_objs, std::ofstream::out | std::ios::app);
		std::string objs_header = "timestamp_1,obj_id_1,frame_id_1,box_x_rel,box_y_rel,box_z_rel,x_pos_rel,y_pos_rel,z_pos_rel,yaw_rel,vx,vy,vz,x_ang,y_ang,z_ang," 
		"blank_1,timestamp_2,frame_id_2,obj_id2,box_x_fixed,box_y_fixed,box_z_fixed,x_pos_fixed,y_pos_fixed,z_pos_fixed,yaw_fixed,blank_2," 
		"obj_id_3,lat,lon,alt";
		outputfile << objs_header << std::endl;
		outputfile.close();
		
		ROS_INFO("Ready");
		ROS_INFO("%s", result_file_path_objs.c_str());
		ROS_INFO("%s", result_file_path_ego.c_str());
		ROS_INFO("%s", objects_topic.c_str());
		
		ros::Subscriber veh_vel_sub = n.subscribe("/current_pose", 1000, &dumpResultsNode::poseCB, this);	
		ros::Subscriber obj_detected_sub = n.subscribe("/current_velocity", 1000, &dumpResultsNode::velCB, this);
		ros::Subscriber veh_pos_sub = n.subscribe(objects_topic, 1000, &dumpResultsNode::detectedObjectCB, this);		
		
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
