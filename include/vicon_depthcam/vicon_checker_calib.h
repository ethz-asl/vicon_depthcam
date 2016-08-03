#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include <kindr/minimal/quat-transformation.h>

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>

class ViconCheckerCalib {
 public:
  ViconCheckerCalib(const ros::NodeHandle& nh,
                    const ros::NodeHandle& private_nh);

  void checker_vicon_callback(
      const geometry_msgs::TransformStamped::ConstPtr& msg);
  void sensor_vicon_callback(
      const geometry_msgs::TransformStamped::ConstPtr& msg);
  void checker_sensor_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

 private:
  typedef kindr::minimal::QuatTransformation::TransformationMatrix TformMat;

  void calc_transform_chain();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  size_t count_;
  TformMat summed_values_;

  ros::Subscriber checker_vicon_sub_;
  ros::Subscriber sensor_vicon_sub_;
  ros::Subscriber checker_sensor_sub_;

  ros::ServiceServer save_tform_service_;

  tf::TransformBroadcaster tf_broadcaster_;

  // world to vicon-sensor-body tform
  kindr::minimal::QuatTransformation T_W_VSB_;
  // world to vicon-checkerboard-body tform
  kindr::minimal::QuatTransformation T_W_VCB_;
  // vicon-checkerboard-body to sensor-checkerboard-body tform
  kindr::minimal::QuatTransformation T_VCB_CB_;
  // sensor-body to sensor-checkerboard-body tform
  kindr::minimal::QuatTransformation T_SB_CB_;
  // sensor-pointcloud-body to sensor-body tform
  kindr::minimal::QuatTransformation T_PB_SB_;
};