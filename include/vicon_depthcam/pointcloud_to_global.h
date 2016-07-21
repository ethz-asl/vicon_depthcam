#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_xml.h"
#include "minkindr_conversions/kindr_tf.h"

/*
 * This Node converts the pointcloud from the IR_Camera's Frame to the
 * Global/Vicon Frame
 */

class PCAligner {
 public:
  
  PCAligner(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

  void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);

  void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg_in);

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pc_pub_;
  
  ros::Subscriber pc_sub_;
  ros::Subscriber vicon_sub_;

  // world to vicon-body
  kindr::minimal::QuatTransformation T_W_VB_;
  // vicon-body to sensor body
  kindr::minimal::QuatTransformation T_VB_SB_;

  tf::TransformBroadcaster tf_broadcaster_;
};