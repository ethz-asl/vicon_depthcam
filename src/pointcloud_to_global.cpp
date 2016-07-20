#include "vicon_depthcam/pointcloud_to_global.h"

void PCAligner::vicon_callback(
    const geometry_msgs::TransformStamped::ConstPtr& input) {
  transformMsgToKindr(cmsg.transform, &T_W_VB_);
}

void PCAligner::pc_callback(const sensor_msgs::PointCloud2ConstPtr& input) {
  sensor_msgs::PointCloud2 output;

  // world to sensor-body transform
  kindr::minimal::QuatTransformation T_W_SB = T_W_VB_ * T_VB_SB_;

  tf::Transform tf_transform;
  tf::transformKindrToTF(T_W_SB, &tf_transform);
  br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "world",
                                        "pc_sensor"));

  // Tranbsform into world (vicon) frame
  pcl_ros::transformPointCloud("world", world_ircamera, *input, output);

  pc_publisher_.publish(output);
}

PCAligner::PCAligner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& private_nh) {
  transformation = Stored_Transformation();
  pc_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("pc_W", 1);
  pc_sub_ = nh_.subscribe("pc_SB", 1, &PCAligner::pc_callback, this);

  vicon_sub_ = nh_.subscribe("vicon_tf", 1, &PCAligner::vicon_callback, this);

  XmlRpc::XmlRpcValue T_VB_SB_xml;
  if (nh_private_.getParam("T_VB_SB", T_VB_SB_xml)) {
    kindr::minimal::xmlRpcToKindr(T_VB_SB_xml, &T_VB_SB_);
  } else {
    ROS_FATAL("Could not find T_VB_SB parameter, exiting");
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_to_global");

  PCAligner pc_aligner;

  ros::spin();

  return 0;
}
