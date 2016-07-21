#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "std_srvs/Empty.h"

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <vector>
#include "find_closest_message.cpp"
#include "minkindr_ext.cpp"
#include "Statistics.cpp"

/*
 * This Node listens to the Messages of the Vicon (Checkerboard & Kinect) and to the Chessboard Detection
 * and calculates the Transform between the Vicon-Kinect-Frame and the IR_Camera's frame.
 *
 * Use the "calc_statistics" in order to print the final averaged transformation.
 */

class Transform
{
public:

	bool calc_statistics(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
	{
		ROS_INFO("rX:");
		print_stats(v_rX);
		ROS_INFO("rY:");
		print_stats(v_rY);
		ROS_INFO("rZ:");
		print_stats(v_rZ);
		ROS_INFO("dx:");
		print_stats(v_dx);
		ROS_INFO("dy:");
		print_stats(v_dy);
		ROS_INFO("dz:");
		print_stats(v_dz);
		ROS_INFO("Average quaternion:");
		//ROS_INFO_STREAM(R_S_I_avg.vector());
		return true;
	}

	void checker_vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& input)
	{
		transformMsgToKindr(input.transform, &T_W_VCB_);
	}

	void sensor_vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& input)
	{
		transformMsgToKindr(input.transform, &T_W_VSB_);
	}

	void checker_camera_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
	{
		poseMsgToKindr(input.transform, &T_SB_CB_);

		calc_diff();


	}

	void calc_diff()
	{

		T_VSB_SB =T_W_VSB_.inverse() * T_W_VCB_ * T_VCB_CB_ * T_SB_CB_.inverse()

		count++;

		//average
		summed_rot_ += T_VSB_SB.getRotation();
		summed_trans += T_VSB_SB.getTranslation();

		mean_T_VSB_SB_ = QuatTransformation(summed_rot_, summed_trans/count);

		ROS_INFO_STREAM("Current Transformation: " << mean_T_VSB_SB_.getTransformationMatrix());

		// tf broadcasts
		tf::Transform world_checker;
		tf::Transform world_sensor;
		tf::Transform checker_detected;
		tf::Transform sensor_ircamera;
		//tf::Transform ircamera_detected;
		//tf::Transform ircamera_detected_backtrans;
		tf::Transform world_ircamera;


		static tf::TransformBroadcaster br;
		tf::transformKindrToTF(T_W_C,&world_checker);
		tf::transformKindrToTF(T_W_S,&world_sensor);
		tf::transformKindrToTF(T_C_D,&checker_detected);
		tf::transformKindrToTF(T_S_I,&sensor_ircamera);
		//tf::transformKindrToTF(T_I_D,&ircamera_detected);
		//tf::transformKindrToTF(T_D_I,&ircamera_detected_backtrans);
		tf::transformKindrToTF(T_W_I,&world_ircamera);

		//world_sensor.



		br.sendTransform(tf::StampedTransform(world_checker, ros::Time::now(), "world", "vicon_checker"));
		br.sendTransform(tf::StampedTransform(world_sensor, ros::Time::now(), "world", "vicon_sensor"));
		br.sendTransform(tf::StampedTransform(checker_detected, ros::Time::now(), "vicon_checker", "checkerboard"));
		br.sendTransform(tf::StampedTransform(sensor_ircamera, ros::Time::now(), "vicon_sensor", "ir_camera"));
		//br.sendTransform(tf::StampedTransform(ircamera_detected, ros::Time::now(), "ir_camera", "ir_detected"));
		//br.sendTransform(tf::StampedTransform(ircamera_detected_backtrans, ros::Time::now(), "checkerboard", "ir_backtranspose"));
		br.sendTransform(tf::StampedTransform(world_ircamera, ros::Time::now(), "world", "ir_direct"));
	}


	Transform() {
		detected_checker_pos= {0.07, 0.07, 0.0};
		detected_checker_rot= {0.0, 0.7071067811865475, 0.7071067811865476, 0.0};
		sum_sensor_ircamera << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
		sensor_ircamera << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
		count=0;
		lib_size=50;
		checker_index=0;
		sensor_index=0;
		checker_vicon_sub=n.subscribe("vrpn_client_chessboard/raw_transform", 1, &Transform::callback_checker_vicon, this);
		sensor_vicon_sub=n.subscribe("vrpn_client_kinect/raw_transform", 1, &Transform::callback_sensor_vicon, this);
		checker_detection_sub=n.subscribe("objectdetection_pose", 1, &Transform::callback_checker_detection, this);
		service= n.advertiseService("print_transform", &Transform::calc_statistics, this);

	}
private:
	ros::NodeHandle n;
	ros::Subscriber checker_vicon_sub;
	ros::Subscriber sensor_vicon_sub;
	ros::Subscriber checker_detection_sub;
	ros::ServiceServer service;

	std::vector<double> v_rX,v_rY,v_rZ,v_dx,v_dy,v_dz;
	kindr::minimal::RotationQuaternion R_S_I_avg;

	kindr::minimal::QuatTransformation anchor;
	Eigen::Matrix<double, 3, 1> detected_checker_pos;
	Eigen::Quaternion<double> detected_checker_rot;
	Eigen::Matrix<double, 4, 4> sum_sensor_ircamera;
	Eigen::Matrix<double, 4, 4> sensor_ircamera;
	Eigen::Matrix<double, 1, 3> euler_angles;


	std::deque<geometry_msgs::TransformStamped> checker_vicon_msgs;
	std::deque<geometry_msgs::TransformStamped> sensor_vicon_msgs;
	geometry_msgs::PoseStamped checker_detection_msg;

	int count, lib_size, checker_index, sensor_index ;

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "transform_calc");

  Transform TransformObj;

  ros::spin();

  return 0;
}
