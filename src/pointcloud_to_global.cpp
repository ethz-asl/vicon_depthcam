#include "vicon_depthcam/pointcloud_to_global.h"

void PCAligner::vicon_callback(
    const geometry_msgs::TransformStamped::ConstPtr& msg) {
  tf::transformMsgToKindr(msg->transform, &T_W_VB_);
}

void PCAligner::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg_in) {
  sensor_msgs::PointCloud2 msg_out;

  // world to sensor-body transform
  kindr::minimal::QuatTransformation T_W_SB = T_W_VB_ * T_VB_SB_;

  tf::Transform tf_transform;
  tf::transformKindrToTF(T_W_SB, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "world",
                                        "pc_sensor"));

  // Transform into world (vicon) frame
  pcl_ros::transformPointCloud("world", tf_transform, *msg_in, msg_out);

  pc_pub_.publish(msg_out);
}

PCAligner::PCAligner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {

  pc_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("pc_W", 1);
  pc_sub_ = nh_.subscribe("pc_SB", 1, &PCAligner::pc_callback, this);

  vicon_sub_ = nh_.subscribe("vicon_tf", 1, &PCAligner::vicon_callback, this);

  XmlRpc::XmlRpcValue T_VB_SB_xml;
  if (private_nh_.getParam("T_VB_SB", T_VB_SB_xml)) {
    kindr::minimal::xmlRpcToKindr(T_VB_SB_xml, &T_VB_SB_);
  } else {
    ROS_FATAL("Could not find T_VB_SB parameter, exiting");
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_to_global");

  ros::NodeHandle nh, private_nh("~");

  PCAligner pc_aligner(nh, private_nh);

  ros::spin();

  return 0;
}
