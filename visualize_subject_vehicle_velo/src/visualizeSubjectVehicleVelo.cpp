

#include <float.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

class visualizeSubjectVehicleVeloNode
{

private:

	ros::NodeHandle n;
	ros::NodeHandle private_n;
	
	// subscriber
	ros::Subscriber velocity_sub;
	ros::Subscriber position_sub;
	// publisher
	ros::Publisher velo_label_pub;
	
	geometry_msgs::Point position;
	
	void position_cb(const geometry_msgs::PoseStamped &pose_msg)
	{
		position = pose_msg.pose.position;
	}
	
	geometry_msgs::Point get_point()
	{
		return position;
	}
	
	void velo_cb(const geometry_msgs::TwistStamped &twist_msg) {
		
		visualization_msgs::Marker marker_textlabel;
		
		std_msgs::ColorRGBA color_white;
		color_white.r = 1.0f;
		color_white.g = 1.0f;
		color_white.b = 1.0f;
		color_white.a = 1.0f;
		
		marker_textlabel.header.frame_id = "base_link_ground";
		
		marker_textlabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		
		marker_textlabel.scale.z = 1.0;
		
		// convert m/s to km/h
		marker_textlabel.text = std::to_string(twist_msg.twist.linear.x*3.6) + " km/h";
		
		marker_textlabel.pose.position = this->get_point();
		marker_textlabel.pose.orientation.x = 0.0;
		marker_textlabel.pose.orientation.y = 0.0;
		marker_textlabel.pose.orientation.y = 0.0;
		marker_textlabel.pose.orientation.w = 0.0;
		marker_textlabel.pose.position.z += 2.5;
		marker_textlabel.color = color_white;
		marker_textlabel.lifetime = ros::Duration(0.4);
		
		velo_label_pub.publish(marker_textlabel);
	}

public:

  visualizeSubjectVehicleVeloNode():n("~"), private_n("~")
  {
  }
  
  void Run()
  {
		
		ros::Subscriber velocity_sub =
				n.subscribe("/current_velocity", 3, &visualizeSubjectVehicleVeloNode::velo_cb, this);
		ros::Subscriber position_sub =
				n.subscribe("/current_position", 3, &visualizeSubjectVehicleVeloNode::position_cb, this);
				
		velo_label_pub = n.advertise<visualization_msgs::Marker>(
				"/subject_vehicle_velo_markers", 1);
				
		ros::spin();
		

    ROS_INFO("Ready");


  }

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualize_subject_vehicle_velo");
  visualizeSubjectVehicleVeloNode app;
  app.Run();

  return 0;

}
