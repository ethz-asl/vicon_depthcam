/*
 *This node listens to the vicon-kinect transform and publishes the tf of the IR_Camera using the calib_data
 */



#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"
#include "calib_data.cpp"
#include <Eigen/Dense>

class TF_Publisher {
public:

	void callback_vicon_sensor(
			const geometry_msgs::TransformStamped::ConstPtr& input) {
		static tf::TransformBroadcaster br;
		kindr::minimal::QuatTransformation T_W_S;
		tf::transformMsgToKindr(input->transform, &T_W_S);
		kindr::minimal::QuatTransformation T_W_I = T_W_S * T_S_I;
		tf::Transform world_ircamera;
		tf::transformKindrToTF(T_W_I, &world_ircamera);
		br.sendTransform(
				tf::StampedTransform(world_ircamera, input->header.stamp,
						"world", "ir_camera"));

	}

	TF_Publisher() {

		sub_vicon_sensor = n_.subscribe("vrpn_client_kinect/raw_transform", 1,
				&TF_Publisher::callback_vicon_sensor, this);
		transformation = Stored_Transformation();
		T_S_I = kindr::minimal::QuatTransformation(
				transformation.getQuaternion(),
				transformation.getTranslation());
	}

private:
	ros::NodeHandle n_;
	ros::Subscriber sub_vicon_sensor;

	geometry_msgs::TransformStamped vicon_sensor_msg;

	Eigen::Matrix<double, 3, 3> temp_rot_matrix;
	Eigen::Matrix<double, 3, 1> temp_translation;
	Eigen::Quaternion<double> temp_quat;
	kindr::minimal::QuatTransformation T_S_I;
	Stored_Transformation transformation;
};

int main(int argc, char **argv) {

	ros::init(argc, argv, "pointcloud_to_global");

	TF_Publisher tf_pub;

	ros::spin();

	return 0;
}
