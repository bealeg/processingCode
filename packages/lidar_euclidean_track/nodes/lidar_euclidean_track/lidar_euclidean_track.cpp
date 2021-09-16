#include <float.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include <std_msgs/ColorRGBA.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

class lidarEuclideanTrackerNode
{
private:

  ros::NodeHandle n;
	ros::NodeHandle private_n;
	
  ros::Subscriber cluster_centroids_sub;
  
  ros::Publisher tracked_pub;
	ros::Publisher tracked_bba_pub;
	ros::Publisher tracked_bba_textlabel_pub;

  std::string         input_point_topic_;
  std::string         target_frame_;
  std::string         output_point_topic_;

  tf::TransformListener *tf_listener_ptr_;
  
  int color_val;
  double threshold_dist;
  std::vector<autoware_msgs::CloudCluster> v_pre_cloud_cluster;

	double euclid_distance(const geometry_msgs::Point pos1, const geometry_msgs::Point pos2) {
		
		return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) +
              pow(pos1.z - pos2.z, 2));
	}
	
	double euclid_distance(const geometry_msgs::PointStamped& pos1, const geometry_msgs::PointStamped& pos2) {
		
		return sqrt(pow(pos1.point.x - pos2.point.x, 2) + pow(pos1.point.y - pos2.point.y, 2) +
              pow(pos1.point.z - pos2.point.z, 2));
	}

	void pos_stamped2pos(geometry_msgs::PointStamped in_pos, geometry_msgs::Point &out_pos) {
		
		out_pos = in_pos.point;
	}
	
	void cluster_cb(const autoware_msgs::CloudClusterArray::Ptr &cloud_cluster_array_ptr) {
	
		tf::StampedTransform local2global_;
		
		
		//input_header_ = cloud_cluster_array_ptr.header;
		autoware_msgs::CloudClusterArray base_msg = *cloud_cluster_array_ptr;
		
		autoware_msgs::CloudClusterArray transformed_input;
		
		static int id = 1;
		
		// for each cluster in the cluster array
		for (int i(0); i < (int)base_msg.clusters.size(); ++i) {
			
			// allocate space for new cluster centroids
			geometry_msgs::Point cluster_centroid;
			double min_distance = DBL_MAX;
			bool stamped_id = false;
			// call function that populates cluster centroid with current
			// cluster centroid from the list of clusters (not stamped)
			pos_stamped2pos(base_msg.clusters.at(i).centroid_point, cluster_centroid);
			
			//std::cout << std::to_string(base_msg.header.stamp.toSec()) << std::endl;
			
			// pre cloud cluster is a static class variable defined at the top
			// loop through the precloud cluster array
			for (int j(0); j < (int)v_pre_cloud_cluster.size(); ++j) {
				
				// allocate space for previous cluster, get data
				geometry_msgs::Point pre_cluster_centroid;
				pos_stamped2pos(v_pre_cloud_cluster.at(j).centroid_point,
												pre_cluster_centroid);
				// get distance from new cluster to the old cluster
				double distance = euclid_distance(cluster_centroid, pre_cluster_centroid);
				// compare distances, if the new cluster distance is within the 
				// threshold, it's accepted as the new cluster
				if (distance < min_distance && distance < threshold_dist) {
					
					// change distance to this new minimum
					min_distance = distance;
					base_msg.clusters.at(i).id = v_pre_cloud_cluster.at(j).id;
					stamped_id = true;
					// at the end of this, should have the closest point to the
					// previous centroid (not an algo that will produce 100%
					// accurate results, but good enough for now
				}
			}
			
			if (!stamped_id) {
				base_msg.clusters.at(i).id = id;
				++id;
				//if (id > 100)
					//id = 1; //don't want these looping back to 1 like they had it
			}

		}
		// try and transform the pointclouds here to global, find the matching
		// id's and calculate a distance, then velo from that. That way we can
		// have a velo calculation in the global sense and not just relative 
		// to the ego vehicle
		
		//std::cout << "x: " << std::to_string(local2global_.getOrigin().x()) << "y: " << std::to_string(local2global_.getOrigin().y()) <<std::endl;
		
		
		for (int i(0); i < (int)base_msg.clusters.size(); ++i){
			//std::cout << object.header.frame_id << std::endl;
			// transform the centroid point to global map frame
			
			geometry_msgs::PointStamped out_point;
			out_point.header = base_msg.header;
			out_point.header.frame_id = "map";
			
			for (int j(0); j < (int)v_pre_cloud_cluster.size(); ++j){
				double dist;
				if (base_msg.clusters.at(i).id == v_pre_cloud_cluster.at(j).id){

					
					
					geometry_msgs::PointStamped pre_out_point;
					out_point.header = v_pre_cloud_cluster.at(j).header;
					out_point.header.frame_id = "map";
					
					try
					{
						//tf_listener_.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
						tf_listener_ptr_->lookupTransform("map", "base_link_ground", base_msg.header.stamp, local2global_);
						tf_listener_ptr_->transformPoint("map", base_msg.header.stamp, base_msg.clusters.at(i).centroid_point, "map", out_point);
					}
					catch (tf::TransformException ex)
					{
						ROS_ERROR("%s", ex.what());
					}
					
					
					try
					{
						//tf_listener_.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
						tf_listener_ptr_->lookupTransform("map", "base_link_ground", ros::Time::now()-ros::Duration(0.1), local2global_);
						tf_listener_ptr_->transformPoint("map", v_pre_cloud_cluster.at(j).header.stamp, v_pre_cloud_cluster.at(j).centroid_point, "map", pre_out_point);
					}
					catch (tf::TransformException ex)
					{
						ROS_ERROR("%s", ex.what());
					}
					//ROS_INFO("pre x: %f y: %f z: %f\n", in_point.point.x, in_point.point.y, in_point.point.z);
					//ROS_INFO("centroid point from pre frame_id : %s x: %f y: %f z: %f\n", v_pre_cloud_cluster.at(j).header.frame_id.c_str(), v_pre_cloud_cluster.at(j).centroid_point.point.x,
									//v_pre_cloud_cluster.at(j).centroid_point.point.y, v_pre_cloud_cluster.at(j).centroid_point.point.z);
									
					dist = this->euclid_distance(out_point,pre_out_point);
					if (v_pre_cloud_cluster.at(j).id == 1) {
						//ROS_INFO("distance moved: %f\n", dist);
						//ROS_INFO("pre x: %f y: %f", pre_out_point.point.x, pre_out_point.point.y);
						//ROS_INFO("pos x: %f y: %f", out_point.point.x, out_point.point.y);
					}
					base_msg.clusters.at(i).score = dist/0.1; // approx timestep, can use for now
				}
			}
		}
		
		
		// clear and then write the current cloud to the precloud
		// transformed_input.clear();
		v_pre_cloud_cluster.clear();
		for (int i(0); i < (int)base_msg.clusters.size(); ++i) {
			autoware_msgs::CloudCluster cloud_cluster = base_msg.clusters.at(i);
			cloud_cluster.header = base_msg.header;
			cloud_cluster.centroid_point.header = base_msg.header;
			cloud_cluster.centroid_point.header.frame_id = base_msg.header.frame_id;
			v_pre_cloud_cluster.push_back(cloud_cluster);
		}

		autoware_msgs::DetectedObjectArray detected_objects_msg;
		detected_objects_msg.header = base_msg.header;
		//for (auto i = base_msg.clusters.begin(); i != base_msg.clusters.end(); i++) {
		for (int i(0); i < (int)base_msg.clusters.size(); ++i) {/*
			autoware_msgs::DetectedObject detected_object;
			detected_object.header = i->header;
			detected_object.id = i->id;
			detected_object.label = i->label;
			detected_object.space_frame = "base_link_ground";
			detected_object.dimensions = i->dimensions; //bounding_box.dimensions;
			detected_object.pose = i->bounding_box.pose;
			detected_objects_msg.objects.push_back(detected_object); */
			autoware_msgs::DetectedObject detected_object;
			detected_object.header = base_msg.clusters.at(i).header;
			detected_object.id = base_msg.clusters.at(i).id;
			detected_object.velocity.linear.x = base_msg.clusters.at(i).score; // this is VELOCITY
			detected_object.label = base_msg.clusters.at(i).label;
			detected_object.space_frame = "base_link_ground";
			detected_object.dimensions = base_msg.clusters.at(i).dimensions; //bounding_box.dimensions;
			detected_object.pose = base_msg.clusters.at(i).bounding_box.pose;
			detected_object.valid = true;
			detected_object.pose_reliable = true;
			detected_object.velocity_reliable = true;
			std_msgs::ColorRGBA color;
			// two options, make them either blue or green
			if (color_val == 1) {
			color.r = 58./255;
			color.g = 124./255;
			color.b = 204./255; // a nice shade of blue
			color.a = 0.8;
		}
		else {
			color.r = 38./255;
			color.g = 178./255;
			color.b = 88./255;
			color.a = 0.8; // nice green color
		}
		detected_object.color = color; //assign color
			detected_objects_msg.objects.push_back(detected_object);
		}

		tracked_pub.publish(detected_objects_msg);

		jsk_recognition_msgs::BoundingBoxArray pub_bb_msg;
		pub_bb_msg.header = base_msg.header;
		for (int i(0); i < (int)base_msg.clusters.size(); ++i) {
			pub_bb_msg.boxes.push_back(base_msg.clusters.at(i).bounding_box);
			pub_bb_msg.boxes.at(i).value = base_msg.clusters.at(i).id;
		}
		tracked_bba_pub.publish(pub_bb_msg);

		visualization_msgs::MarkerArray pub_textlabel_msg;
		std_msgs::ColorRGBA color_white;
		color_white.r = 1.0f;
		color_white.g = 1.0f;
		color_white.b = 1.0f;
		color_white.a = 1.0f;
		for (int i(0); i < (int)base_msg.clusters.size(); ++i) {
			visualization_msgs::Marker marker_textlabel;
			marker_textlabel.header.frame_id = "base_link_ground";  // "velodyne //original
			marker_textlabel.ns = "cluster";
			marker_textlabel.id = i;
			/* Set the text label */
			marker_textlabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker_textlabel.scale.z = 1.0;
			marker_textlabel.text = base_msg.clusters.at(i).label + "@" +
															std::to_string(base_msg.clusters.at(i).id);
			marker_textlabel.pose.position =
					base_msg.clusters.at(i).bounding_box.pose.position;
			marker_textlabel.pose.orientation.x = 0.0;
			marker_textlabel.pose.orientation.y = 0.0;
			marker_textlabel.pose.orientation.y = 0.0;
			marker_textlabel.pose.orientation.w = 0.0;
			marker_textlabel.pose.position.z += 1.5;
			marker_textlabel.color = color_white;
			marker_textlabel.lifetime = ros::Duration(0.4);
			pub_textlabel_msg.markers.push_back(marker_textlabel);
		}
		tracked_bba_textlabel_pub.publish(pub_textlabel_msg);

	}

public:

  lidarEuclideanTrackerNode(tf::TransformListener* in_tf_listener_ptr):n("~"), private_n("~")
  {
    tf_listener_ptr_ = in_tf_listener_ptr;
  }
  
  void Run()
  {

		if (!private_n.getParam("threshold_dist", threshold_dist)) {
			threshold_dist = 2.0;
		}
		if (!private_n.getParam("color_val", color_val)) {
			color_val = 1;
		}
		ros::Subscriber cluster_centroids_sub =
				n.subscribe("/detection/lidar_detector/cloud_clusters", 3, &lidarEuclideanTrackerNode::cluster_cb, this);
		tracked_pub =
				n.advertise<autoware_msgs::DetectedObjectArray>("/detection/object_tracker/objects", 1);
		tracked_bba_pub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>(
				"/cloud_cluster_tracked_bounding_box", 1);
		tracked_bba_textlabel_pub = n.advertise<visualization_msgs::MarkerArray>(
				"/cloud_cluster_tracked_text", 1);
				
		ros::spin();

    //ROS_INFO("Subscribing to... %s", input_point_topic_.c_str());
		//points_node_sub_ = node_handle_.subscribe(input_point_topic_, 1, &CloudTransformerNode::CloudCallback, this);

    ROS_INFO("Ready");


  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "euclidean_lidar_track");
  tf::TransformListener tf_listener;
  lidarEuclideanTrackerNode app(&tf_listener);

  app.Run();

  return 0;

}
