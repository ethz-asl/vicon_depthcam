#include "vicon_depthcam/vicon_checker_calib.h"

ViconCheckerCalib::ViconCheckerCalib(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), count_(0), summed_values_(TformMat::Zero()) {
  XmlRpc::XmlRpcValue T_VCB_CB_xml;
  if (private_nh_.getParam("T_VCB_CB", T_VCB_CB_xml)) {
    kindr::minimal::xmlRpcToKindr(T_VCB_CB_xml, &T_VCB_CB_);
  } else {
    ROS_FATAL("Could not find T_VCB_CB parameter, exiting");
    exit(EXIT_FAILURE);
  }

  checker_vicon_sub_ =
      nh_.subscribe("checker_vicon_tform", 1,
                    &ViconCheckerCalib::checker_vicon_callback, this);
  sensor_vicon_sub_ = nh_.subscribe(
      "sensor_vicon_tform", 1, &ViconCheckerCalib::sensor_vicon_callback, this);
  checker_sensor_sub_ =
      nh_.subscribe("sensor_camera_pose", 1,
                    &ViconCheckerCalib::checker_sensor_callback, this);
  // save_tform_service_ =
  //    nh_.advertiseService("save_transform", &Transform::calc_statistics,
  //    this);
}

void ViconCheckerCalib::checker_vicon_callback(
    const geometry_msgs::TransformStamped::ConstPtr& msg) {
  tf::transformMsgToKindr(msg->transform, &T_W_VCB_);
}

void ViconCheckerCalib::sensor_vicon_callback(
    const geometry_msgs::TransformStamped::ConstPtr& msg) {
  tf::transformMsgToKindr(msg->transform, &T_W_VSB_);
}

void ViconCheckerCalib::checker_sensor_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  tf::poseMsgToKindr(msg->pose, &T_SB_CB_);

  calc_transform_chain();
}

void ViconCheckerCalib::calc_transform_chain() {
  kindr::minimal::QuatTransformation T_VSB_SB =
      T_W_VSB_.inverse() * T_W_VCB_ * T_VCB_CB_ * T_SB_CB_.inverse();

  count_++;
  summed_values_ += T_VSB_SB.getTransformationMatrix();

  kindr::minimal::RotationQuaternion temp_rot_quat =
      kindr::minimal::RotationQuaternion::constructAndRenormalize(
          summed_values_.topLeftCorner(3, 3) / count_);

  kindr::minimal::QuatTransformation mean_T_VSB_SB(
      temp_rot_quat, summed_values_.topRightCorner(3, 1) / count_);

  ROS_INFO_STREAM("Processed " << count_ << " images, current transformation: ");
  ROS_INFO_STREAM(mean_T_VSB_SB.getTransformationMatrix());

  // tf broadcasts
  tf::Transform tf_transform;
  tf::transformKindrToTF(T_W_VSB_, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_transform, ros::Time::now(), "world", "vicon_sensor_body"));
  tf::transformKindrToTF(T_W_VCB_, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_transform, ros::Time::now(), "world", "vicon_checkerboard_body"));
  tf::transformKindrToTF(T_SB_CB_, &tf_transform);
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_transform, ros::Time::now(), "sensor_body",
                           "image_checkerboard_body"));
  tf::transformKindrToTF(T_VSB_SB, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_transform, ros::Time::now(), "vicon_sensor_body", "sensor_body"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "transform_calc");

  ros::NodeHandle nh, private_nh("~");

  ViconCheckerCalib vicon_checker_calib(nh, private_nh);

  ros::spin();

  return 0;
}
