#include <boost/filesystem.hpp>
#include <chrono>
#include <vector>
#include <stdio.h>
#include <string>
#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;


class rosbagWork
{
private:
    
    ros::NodeHandle nh;
    ros::NodeHandle private_n;
    
    std::string input_bag_file;
    std::string output_bag_file;
    std::string target_frame;
    bool write_bool;
    ros::Time thisTime;
    
    std::string lidar_front_left_topic;
    std::string lidar_front_right_topic;
    std::string lidar_rear_topic;
    std::string lidar_rear_tf = "lidar_rear";
    std::vector<std::string> pc_topics;
    
public:

    rosbagWork():nh("~"), private_n("~") {
        // params
        if(!nh.getParam("input_bag_file",input_bag_file)){
            input_bag_file = "/home/gbeale/Downloads/Bags/BoltOn/ShortBag2.bag";
        }
        if(!nh.getParam("output_bag_file",output_bag_file)){
            output_bag_file = "/home/gbeale/Downloads/Bags/BoltOn/output.bag";
        }
        if(!nh.getParam("write_bool",write_bool)){
            write_bool = false;
        }
        if(!nh.getParam("target_frame",target_frame)){
            target_frame = "base_link_ground";
        }
        
        lidar_front_left_topic = "/lidar_front_left/velodyne_points";
        lidar_front_right_topic = "/lidar_front_right/velodyne_points";
        lidar_rear_topic = "/lidar_rear/velodyne_points";
        pc_topics.push_back(lidar_front_left_topic);
        pc_topics.push_back(lidar_front_right_topic);
        pc_topics.push_back(lidar_rear_topic);
        
        
        ROS_INFO("[rosbagWork] Input file location: %s", input_bag_file.c_str());
        ROS_INFO("[rosbagWork] Onput file location: %s", output_bag_file.c_str());
        ROS_INFO("[rosbagWork] Write to file?: %s ", write_bool ? "true" : "false");
        ROS_INFO("[rosbagWork] Target frame: %s", target_frame.c_str());
        
        
    }
  
    void Run()
    {
        // log
        ROS_INFO("[rosbagWork] Begin Initialization");
        
        rosbag::Bag bag_in;
        rosbag::Bag bag_out;
        bag_out.setCompression(rosbag::compression::BZ2);
        
        try {
            bag_in.open(input_bag_file, rosbag::bagmode::Read);
        } catch (const rosbag::BagException&) {
            ROS_INFO("[rosbagWork] Error opening file %s", input_bag_file.c_str());
        }
        
        try {
            bag_out.open(output_bag_file,rosbag::bagmode::Write);
        } catch (const rosbag::BagException&) {
             ROS_INFO("[rosbagWork] Error opening file %s", output_bag_file.c_str());
        }
        
        
        rosbag::View view(bag_in,rosbag::TopicQuery(pc_topics));
        rosbag::View::iterator view_it;
        
        
        ros::Time bag_start_time = view.getBeginTime();
        ros::Time bag_end_time = view.getEndTime();
        ros::Duration duration = bag_end_time - bag_start_time;
        
        ROS_INFO("[rosbagWork] Bag start time: %f", bag_start_time.toSec());
        ROS_INFO("[rosbagWork] Bag end time: %f", bag_end_time.toSec());
        ROS_INFO("[rosbagWork] Duration: %f", duration.toSec());
        
        tf2_ros::Buffer tfBuffer(duration);
        
        // TF
        tf::TransformListener tf_listener;
        tf::TransformBroadcaster tf_broadcaster;
        
        /*
        rosbag::View topic_list_view(bag_in);
        std::string target_topic;
        std::map<std::string, std::string> topic_list;
        for(rosbag::ConnectionInfo const *ci: topic_list_view.getConnections() )
        {
            topic_list[ci->topic] = ci->datatype;
            for(decltype(pc_topics.size()) i = 0; i <= pc_topics.size() - 1; i++)
            {
                if (ci->topic == pc_topics[i])
                {
                    if (ci->datatype == std::string("sensor_msgs/PointCloud2"))
                    {
                        // target_topic = std::string (argv[2]);
                        ROS_INFO("[rosbagWork] Adding topic %s to bag query",pc_topics[i].c_str());
                        view.addQuery (bag_in, rosbag::TopicQuery (pc_topics[i]));
                    }
                    else
                    {
                        ROS_INFO("[rosbagWork] Provided topic '%s' is in the bag file, but is not of type sensor_msgs/PointCloud2 (type: %s)",pc_topics[i].c_str(),ci->datatype.c_str());
                    }
                }
            }
        }
        */
        
        view.addQuery(bag_in, rosbag::TypeQuery ("tf/tfMessage"));
        view.addQuery(bag_in, rosbag::TypeQuery ("tf2_msgs/TFMessage"));
        view_it = view.begin();
        
        std::string auth = "bag_work";
        
        PointCloud cloud_t ;
        // PointCloudConstPtr cloud;
        ros::Duration r (0.001);
        // Loop over the whole bag file
        while (view_it != view.end ())
        {
            // Handle TF messages first
            ROS_INFO("%s",view_it->getTopic().c_str());
            tf::tfMessage::ConstPtr tf = view_it->instantiate<tf::tfMessage> ();
            if (tf != NULL) {
                ROS_INFO("[rosbagWork] sending tf message");
                //tf_broadcaster.sendTransform (tf->transforms);
                for(decltype(tf->transforms.size()) i = 0; i <= tf->transforms.size() - 1; i++) {
                    tfBuffer.setTransform(tf->transforms[i],auth);
                }
                //tfBuffer.setTransform(tf->transforms,auth);
                // ros::spinOnce();
                // r.sleep();
            }
            else {
                
                // Get the PointCloud2 message
                PointCloudConstPtr cloud = view_it->instantiate<PointCloud> ();
                if (cloud == NULL) {
                    ++view_it;
                    continue;
                }
                cloud_t = *cloud;
                ROS_INFO("[rosbagWork] Got %d data points from topic %s on frame: %s", cloud_t.width * cloud_t.height, view_it->getTopic().c_str(), cloud_t.header.frame_id.c_str());
                
                // If a target_frame was specified
                if(strcmp(view_it->getTopic().c_str(),lidar_rear_topic.c_str()) == 0) {
                    
                    // Transform it
                    geometry_msgs::TransformStamped trans;
                    
                    try {
                        trans = tfBuffer.lookupTransform(lidar_rear_tf, target_frame,ros::Time(0));
                        //pcl_ros::transformPointCloud (target_frame,*cloud, cloud_t, tf_listener);
                        // ROS_INFO("[rosbagWork] Got %d data points from topic %s. Old frame %s. New frame is now %s", cloud_t.width * cloud_t.height, view_it->getTopic().c_str(), *cloud->header.frame_id.c_str(),cloud_t.header.frame_id.c_str());
                        //if (!pcl_ros::transformPointCloud (target_frame, *cloud, cloud_t, tf_listener)){
                            //++view_it;
                            //continue;
                        //}
                    } 
                    catch (tf2::ExtrapolationException &ex) {
                        ROS_INFO("[rosbagWork] Couldn't transform point cloud");
                        ROS_ERROR("%s",ex.what());
                        ++view_it;
                        cloud_t = *cloud;
                        continue;
                    } 
                    catch (tf2::LookupException &ex) {
                        ROS_INFO("[rosbagWork] Couldn't transform point cloud");
                        ROS_ERROR("%s",ex.what());
                        ++view_it;
                        cloud_t = *cloud;
                        continue;
                    }
                    catch (tf::TransformException &ex) {
                        ROS_INFO("[rosbagWork] Couldn't transform point cloud");
                        ROS_ERROR("%s",ex.what());
                        ++view_it;
                        cloud_t = *cloud;
                        continue;
                    }
                }
                // std::cerr << "Got " << cloud_t.width * cloud_t.height << " data points in frame " << cloud_t.header.frame_id << " on topic " << view_it->getTopic() << " with the following fields: " << pcl::getFieldsList (cloud_t) << std::endl;
            }
            // Increment the iterator
            ++view_it;
        }
        bag_in.close();
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_work_cpp");
    // tf::TransformListener tf_listener;
    rosbagWork app;
    app.Run();

    return 0;

}
