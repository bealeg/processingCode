#include <boost/filesystem.hpp>
#include <vector>
#include <stdio.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/io/concatenate_data.h>
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Time.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*
 * rosbag_work_simple.cpp
 * Created by Gregory Beale, January 26, 2023
 * 
 * This ros script is responsible for concatenating the front left and front right lidars
 * for the Bolt On projects, as well as trtansforming the rear lidar to the base_link_ground frame. 
 * Bag recordings are already available on the network, and this script reads a given bag file in, 
 * looks for the corresponding transform and point cloud messages on specified topics, 
 * matches front left and front right lidar point clouds by timestamp, transforms them to the 
 * base_link_ground frame, concatenates them together into one cloud, and writes them to the ouput
 * bag file if instructed to do so. The rear lidar is also transformed into the matching frame and
 * written to the output bag file along with all the tf messages received.
 * 
 * There are two point cloud print statements (currently disabled) meant to serve as a visual check 
 * on the number of points being transformed and concatenated, as well as the new frames for each 
 * transformation. All point clouds are eventually transformed into the base_link_ground frame.
 * 
 * Many values, such as which topics to read and write to have been hard coded in this script
 * to work specifically with the Bolt On recordings. Additionally, because I was having trouble
 * getting the transform listener to store static and non-static transforms that the bag reader
 * sees for every timestep, I have hard coded the static transforms in as variables by looking
 * them up in the rosbag using tf_echo outside of this script. This is useful here because
 * there is only one non-static transform (the calculated rotation of the kingpin frame). There is 
 * probably a way to get the tf listener to have all the necessary transforms in the buffer, but I 
 * could not find a way, and the current method works. If the static transforms change these values
 * will need to be changed. One improvement could be to get the known static transforms from
 * the tf messages (usually the first stored in the bag) and save them to the static variables.
 * This way the tf's wouldn't need to be checked by hand and hard coded in.
 * 
 * The current run time is limited mostly by the write speed to disk and the ability to read chunks
 * from the network. Best performance is expected on the cluster with the large files (> 75GB), but 
 * local speeds are about 2x times slower than the orignal time length of the recording.
 * 
 * To build, follow normal ROS build mechanics with catkin and name of package.
 * catkin build tf_pointcloud
 * 
 * Run using the 'rosrun' command followed by the package name and then the parameters.
 * 
 * Paramters
 *  <file_in.bag> <output_directory> <file_name> <write_bool>
 * 
 *  file_in          -> path to original bag
 *  output_directory -> path of directory for output
 *  file_name        -> name of output file (does not include path)
 *  write_bool       -> boolean flag to write to new file
 * 
 *  example: rosrun tf_pointcloud rosbag_work_cpp_simple /home/gbeale/Downloads/Bags/BoltOn/ShortBag3.bag /home/gbeale/Downloads/Bags/BoltOn output.bag true
 * 
 */

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

/*************************************
 *       Set up static vars
 * ***********************************/

static std::string target_frame;
static tf::Transform transform_left_lidar;
static tf::Transform transform_right_lidar;
static tf::Transform transform_kingpin_rear_lidar;
static tf::Transform transform_fifth_wheel_base_link_ground;
static rosbag::Bag bag_out;
static bool write_bool;
static ros::Time right_cloud_received_time;

static std::string concat_topic; 
static std::string tf_topic;

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {

public:
  void newMessage(const boost::shared_ptr<M const> &msg) {
    this->signalMessage(msg);
  }
};

/*************************************
 * 
 *  Approximate time sync callback.
 *  Transform and concatenate clouds.
 * 
 * ***********************************/
static void point_cloud_sync_cb(const sensor_msgs::PointCloud2::ConstPtr &left_cloud, const sensor_msgs::PointCloud2::ConstPtr &right_cloud) {
    
    // print out for left lidar
    // std::cout << "[rosbagWork] Got " << left_cloud->width * left_cloud->height << " data points in frame " << left_cloud->header.frame_id  << std::endl;
    // print out for right lidar
    // std::cout << "[rosbagWork] Got " << right_cloud->width * right_cloud->height << " data points in frame " << right_cloud->header.frame_id << std::endl;
    
    PointCloudPtr cloud_right_tf (new sensor_msgs::PointCloud2());
    PointCloudPtr cloud_left_tf (new sensor_msgs::PointCloud2());
    PointCloud concat_cloud;
    
    bool left_success = false;
    bool right_success = false;
    
    // try to transfomr left cloud
    try {
      pcl_ros::transformPointCloud (target_frame, transform_left_lidar,*left_cloud, *cloud_left_tf);
      left_success = true;
    } catch (tf::TransformException &ex) {
      std::cerr << "[rosbagWork] Couldn't transform left point cloud." << std::endl;
      std::cerr << ex.what() << std::endl;
    }
    
    // try to transform right cloud
    try {
      pcl_ros::transformPointCloud (target_frame, transform_right_lidar,*right_cloud, *cloud_right_tf);
      right_success = true;
    } catch (tf::TransformException &ex) {
      std::cerr << "[rosbagWork] Couldn't transform left point cloud." << std::endl;
      std::cerr << ex.what() << std::endl;
    }
    
    // concatenate if transforms successful
    if (right_success && left_success) {
      try {
        
        // concatenate clouds, assign header
        pcl::concatenatePointCloud(*cloud_right_tf,*cloud_left_tf, concat_cloud);
        concat_cloud.header.stamp = cloud_right_tf->header.stamp;
        
        // visual debug print out 
        // ******std::cout << "[rosbagWork] Frame changes: [" << left_cloud->header.frame_id << ", " << right_cloud->header.frame_id << "] -> [" << cloud_left_tf->header.frame_id << ", " << cloud_right_tf->header.frame_id << "] -> [" <<  concat_cloud.header.frame_id << "]. Points: [" << left_cloud->width * left_cloud->height << " + " << right_cloud->width * right_cloud->height << "] ?= [" << concat_cloud.width * concat_cloud.height << "]" << std::endl;
        
        if (write_bool) {
          bag_out.write(concat_topic, right_cloud_received_time, concat_cloud);
          // bag_out.write(concat_topic, right_cloud->header.stamp, concat_cloud);
        }
      } catch (...){
        std::cerr << "[rosbagWork] Couldn't concatenate clouds." << std::endl;
      }
    }
    
  }

class RosBagWork
{
public:
  
  void readBag(const std::string &file_in, const std::string &directory, const std::string &file_out_name, const bool &write_bool) {
    
    /*************************************
     *         Bring in parameters
     * ***********************************/
    std::cout << "[rosbagWork] Beginning setup." << std::endl;
    
    // topic information
    std::string lidar_front_left_topic = "/lidar_front_left/velodyne_points";
    std::string lidar_front_right_topic = "/lidar_front_right/velodyne_points";
    std::string lidar_rear_topic = "/lidar_rear/velodyne_points";
    std::vector<std::string> pc_topics;
    pc_topics.push_back(lidar_front_left_topic);
    pc_topics.push_back(lidar_front_right_topic);
    pc_topics.push_back(lidar_rear_topic);
    std::string right_tf_topic = "/lidar_front_right/velodyne_points/tf";
    std::string left_tf_topic = "/lidar_front_left/velodyne_points/tf";
    std::string rear_tf_topic = "/lidar_rear/velodyne_points_transformed";
    concat_topic = "/concatenate_data/output";
    tf_topic = "/tf";
    std::string kingpin_frame = "kingpin";
    std::string fifth_wheel_frame = "fifth_wheel";
    target_frame = "base_link_ground";
    std::string time_topic = "/now_time";
    
    // file directory information for reading and writing
    std::string input_bag_file = file_in;
    std::string output_bag_dir = directory;
    std::string output_bag_file = output_bag_dir + "/" + file_out_name;
    
    std::cout << "[rosbagWork] Input file: " << file_in << std::endl;
    std::cout << "[rosbagWork] Output directory: " << output_bag_dir << std::endl;
    std::cout << "[rosbagWork] Output file name: " << output_bag_file << std::endl;
    std::cout << "[rosbagWork] Write to file? " << write_bool << std::endl;
    std::cout << "[rosbagWork] Output frame: " << target_frame << std::endl;
    
    /*************************************
     *       Hardcode transforms
     * ***********************************/
    // tf::TransformListener tf_listener;
    // tf::TransformBroadcaster tf_broadcaster;
    
    // Putting these tf's in by hand since they are static. For some reason looking up transforms
    // that are a combination of static and non-static transforms does not work very well when reading bag files like this :/
    // There's always some problem with getting them all in the buffer so you can automatically look them up.
    // Ends up throwing a transformLookupException. Very annoying, not sure of fix.
    
    // left front lidar transform (/base_link_ground to /lidar_front_left)
    transform_left_lidar.setOrigin(tf::Vector3(5.433, 1.245, 0.976));
    tf::Quaternion q_left(0.023, -0.012, -0.008, 1.000);
    transform_left_lidar.setRotation(q_left);
    
    // right front lidar transform (/base_link_ground to /lidar_front_right)
    transform_right_lidar.setOrigin(tf::Vector3(5.427, -1.208, 0.951));
    tf::Quaternion q_right(-0.005, 0.021, 0.009, 1.000);
    transform_right_lidar.setRotation(q_right);
    
    // kingpin to rear lidar (/kingpin to /lidar_rear)
    transform_kingpin_rear_lidar.setOrigin(tf::Vector3(-15.332, 0.041, -0.350));
    tf::Quaternion q_kingpin_rear_lidar(-0.003, 0.006, 1.000, 0.000);
    transform_kingpin_rear_lidar.setRotation(q_kingpin_rear_lidar);
    
    // base_link_ground to fifth_wheel
    transform_fifth_wheel_base_link_ground.setOrigin(tf::Vector3(-1.772, 0.028, 1.131));
    tf::Quaternion q_fifth_wheel_base_link_ground(0.000, 0.000, 0.000, 1.000);
    transform_fifth_wheel_base_link_ground.setRotation(q_fifth_wheel_base_link_ground);
    
    // non-static tf from fifth_wheel to kingpin, will be updated as tf's are read
    tf::Transform transform_fifth_wheel_kingpin;
    bool got_fifth_wheel_kingpin_tf = false;
    
    
    /*************************************
     *       Set up message filters
     * ***********************************/
    
    // Set up fake subscribers to capture point clouds, define approximate time sync policy with queue size, bind them to static function
    BagSubscriber<sensor_msgs::PointCloud2> left_lidar_sub, right_lidar_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> concat_clouds_sync(MySyncPolicy(10), left_lidar_sub, right_lidar_sub);
    concat_clouds_sync.registerCallback(boost::bind(&point_cloud_sync_cb, _1, _2));
    
    
    /*************************************
     *     Get input/output bag ready
     * ***********************************/
    rosbag::Bag bag_in;
    rosbag::View::iterator view_it;
    
    // try opening input bag and output bag, set compression
    try
    {
      bag_in.open (file_in, rosbag::bagmode::Read);
    } 
    catch (const rosbag::BagException&) 
    {
      std::cerr << "[rosbagWork] Error opening file " << file_in << std::endl;
    }
    if (write_bool) {
      try {
        bag_out.open(output_bag_file,rosbag::bagmode::Write);
        bag_out.setCompression(rosbag::compression::BZ2);
      } catch (const rosbag::BagException&) {
        std::cerr << "[rosbagWork] Error opening file " << output_bag_file << std::endl;
      }
    }
    
    // create view object and add topic queries, print out bag info
    rosbag::View view(bag_in,rosbag::TopicQuery(pc_topics));
    view.addQuery (bag_in, rosbag::TypeQuery ("tf/tfMessage"));
    view.addQuery (bag_in, rosbag::TypeQuery ("tf2_msgs/TFMessage"));
    view_it = view.begin ();
    
    ros::Time bag_start_time = view.getBeginTime();
    ros::Time bag_end_time = view.getEndTime();
    ros::Duration duration = bag_end_time - bag_start_time;
    
    std::cout << "[rosbagWork] Bag start time: " << bag_start_time.toSec() << std::endl;
    std::cout << "[rosbagWork] Bag end time: " << bag_end_time.toSec() << std::endl;
    std::cout << "[rosbagWork] Duration: " << duration.toSec() << std::endl;
    
    /*************************************
     *       Set up output directory
     * ***********************************/
    
    // create directory if not already there
    boost::filesystem::path outpath (output_bag_dir);
    if (!boost::filesystem::exists (outpath))
    {
      if (!boost::filesystem::create_directories (outpath))
      {
        std::cerr << "[rosbagWork] Error creating directory " << output_bag_dir << std::endl;
      }
      std::cerr << "[rosbagWork] Creating directory " << output_bag_dir << std::endl;
    }
    std::cerr << "[rosbagWork] Saving new bag file to " << output_bag_file << std::endl;
    
    
    /*************************************
     *           Read messages
     * ***********************************/
    PointCloud cloud_t;
    
    // Loop over the whole bag file and read the tf and point cloud messages on the added topics
    while (view_it != view.end ())
    {
      
      // Handle TF messages first, try to instantiate tf message
      tf::tfMessage::ConstPtr tf = view_it->instantiate<tf::tfMessage> ();
      if (tf != NULL)
      {
        // std::cout << "TF time: " << view_it->getTime() << std::endl;
        // tf_broadcaster.sendTransform (tf->transforms);
        if (write_bool) {
          
          // Static tf's have timestamps that are many seconds (~120) before the rest of the tf's.
          // That is okay, it will just make the bag files arbitrarily longer durations (won't really matter with big files)
          // bag_out.write(tf_topic, view_it->getTime(), tf);
        }
        
        // loop through tf's to get the fifth_wheel to kingpin tf
        for(decltype(tf->transforms.size()) i = 0; i <= tf->transforms.size() - 1; i++) {
          
          // std::cout << "[" << tf->transforms[i].header.frame_id << "," << tf->transforms[i].child_frame_id << "] " << tf->transforms[i].header.stamp << std::endl;
          
          // find correct transform
          if (strcmp(tf->transforms[i].child_frame_id.c_str(),kingpin_frame.c_str()) == 0 && strcmp(tf->transforms[i].header.frame_id.c_str(),fifth_wheel_frame.c_str()) == 0) {
            
            // create transform and break out
            transform_fifth_wheel_kingpin.setOrigin(tf::Vector3(tf->transforms[i].transform.translation.x,tf->transforms[i].transform.translation.y,tf->transforms[i].transform.translation.z));
            tf::Quaternion q (tf->transforms[i].transform.rotation.x,tf->transforms[i].transform.rotation.y,tf->transforms[i].transform.rotation.z,tf->transforms[i].transform.rotation.w);
            transform_fifth_wheel_kingpin.setRotation(q);
            
            got_fifth_wheel_kingpin_tf = true;
            break;
          }
        }
        
        /*
        // get rear lidar to base_link_ground
        try {
           tf_listener.lookupTransform("base_link_ground", "lidar_front_left", ros::Time(0), transform_left_lidar);
           std::cout << "[rosbagWork] Successful transfrom lookup for front left lidar." << std::endl;
          } catch (tf::TransformException ex){
            std::cout << "[rosbagWork] Can't look up transform left lidar." << std::endl;
          }*/
        
      }
      else { // message was not a tf message
        
        // try to instantiate point cloud message, if null continue
        PointCloudConstPtr cloud = view_it->instantiate<PointCloud> ();
        if (cloud == NULL) {
          ++view_it;
          continue;
        }
        
        // front right lidar
        if(strcmp(view_it->getTopic().c_str(),lidar_front_right_topic.c_str()) == 0 ) {
            
            // save the received time to use as the write time for the concatenated output
            right_cloud_received_time = view_it->getTime();
            // send message to right cloud subsciber
            right_lidar_sub.newMessage(cloud);
          }
        // front lef lidar
        else if(strcmp(view_it->getTopic().c_str(),lidar_front_left_topic.c_str()) == 0 ) {
            
            // send message to left cloud subscriber
            left_lidar_sub.newMessage(cloud);
        }
        // rear lidar
        else if(strcmp(view_it->getTopic().c_str(),lidar_rear_topic.c_str()) == 0 ) {
          
          // transform based on stored tf's
          try {
            if (got_fifth_wheel_kingpin_tf) {
              
              // tf to kingpin
              pcl_ros::transformPointCloud (kingpin_frame, transform_kingpin_rear_lidar, *cloud, cloud_t);
              
              // tf to fifth_wheel
              pcl_ros::transformPointCloud (fifth_wheel_frame, transform_fifth_wheel_kingpin, cloud_t, cloud_t);
              
              // tf to base_link_ground
              pcl_ros::transformPointCloud (target_frame, transform_fifth_wheel_base_link_ground, cloud_t, cloud_t);
              
              // print out for visual check
              // std::cout << "[rosbagWork] Rear lidar points: " << cloud->width * cloud->height << " ?= " << cloud_t.width * cloud_t.height << " on frame: " << cloud_t.header.frame_id << std::endl;
              
              if (write_bool) {
                // bag_out.write(rear_tf_topic, cloud->header.stamp, cloud_t);
                bag_out.write(rear_tf_topic, view_it->getTime(), cloud_t);
              }
            }
          } catch (tf::TransformException &ex) { // catch transform problem
            std::cerr << "[rosbagWork] Couldn't transform left point cloud." << std::endl;
            std::cerr << ex.what() << std::endl;
            ++view_it;
            continue;
          } catch (...) { // catch anything else
            std::cerr << "[rosbagWork] Couldn't transform left point cloud." << std::endl;
            ++view_it;
            continue;
          }
        }
        
        
      }
      // Increment the iterator
      ++view_it;
      
    } // end while loop that reads bag
    
    // close bag files
    bag_in.close();
    if (write_bool) {bag_out.close();}
  }
};


int main (int argc, char** argv)
{
  
  /*************************************
   *    Read in and store arguments
   * ***********************************/
  
  if (argc < 5) {
    std::cerr << "[rosbagWork] Syntax is: " << argv[0] << " <file_in.bag> <output_directory> <file_name> <write_bool>" << std::endl;
    std::cerr << "[rosbagWork] Example: " << argv[0] << " /path/to/file/in/data.bag path/to/directory output_concat.bag true" << std::endl;
    return (-1);
  }
  
  std::string file_in = std::string(argv[1]);
  std::string dir = std::string(argv[2]);
  std::string file_out_name = std::string(argv[3]);
  if (strcmp(argv[4],"true") == 0) {
    write_bool = true;
  } else {
    write_bool = false;
  }
  
  /*************************************
   *       Call object function
   * ***********************************/
  ros::init (argc, argv, "bag_tf_and_concatenate");
  ros::Time::init();
  
  
  class RosBagWork obj;
  ros::Time start_time = ros::Time::now();
  obj.readBag(file_in, dir, file_out_name, write_bool);
  ros::Time end_time = ros::Time::now();
  
  std::cout << "Finished in: " << end_time - start_time << " seconds." << std::endl;
  
  return(0);
}



