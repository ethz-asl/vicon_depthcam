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

	void callback_checker_vicon(const geometry_msgs::TransformStamped::ConstPtr& input)
	{
		checker_vicon_msgs.push_front(*input);
		if(checker_vicon_msgs.size()>lib_size)
		{
			checker_vicon_msgs.pop_back();
		}
		//count1++;
		//ROS_INFO("checker_vicon_msg number '%i'", count1);
	}

	void callback_sensor_vicon(const geometry_msgs::TransformStamped::ConstPtr& input)
	{
		sensor_vicon_msgs.push_front(*input);
				if(sensor_vicon_msgs.size()>lib_size)
				{
					sensor_vicon_msgs.pop_back();
				}

		//count2++;
		//ROS_INFO("sensor_vicon_msg number '%i'", count2);
	}

	void callback_checker_detection(const geometry_msgs::PoseStamped::ConstPtr& input)
	{
		checker_detection_msg =*input;

		calc_diff();


	}

	void calc_diff()
	{
		checker_index=0;
		sensor_index=0;
		//search closest checker_vicon_msg
		checker_index=nns(checker_detection_msg.header.stamp.toSec(),checker_vicon_msgs);
		//search closest sensor_vicon_msg
		sensor_index=nns(checker_detection_msg.header.stamp.toSec(),sensor_vicon_msgs);

		//S=Sensor, C=Checkerboard, W=World D=Detected Point on Chessboard I=IR-Camera
		kindr::minimal::QuatTransformation T_W_C;
		kindr::minimal::QuatTransformation T_W_S;
		kindr::minimal::QuatTransformation T_I_D;
		kindr::minimal::QuatTransformation T_D_I;
		kindr::minimal::QuatTransformation T_C_D;
		kindr::minimal::QuatTransformation T_S_I;
		kindr::minimal::QuatTransformation T_W_I;

		T_C_D=kindr::minimal::QuatTransformation(detected_checker_rot,detected_checker_pos);

		tf::transformMsgToKindr(checker_vicon_msgs[checker_index].transform, &T_W_C);
		tf::transformMsgToKindr(sensor_vicon_msgs[sensor_index].transform, &T_W_S);
		tf::poseMsgToKindr(checker_detection_msg.pose, &T_I_D);

		T_D_I=T_I_D.inverse();
		T_W_I=(T_W_C*T_C_D)*T_D_I;
		T_S_I=T_W_S.inverse()*T_W_I;

		count++;

		//##################Calculate Average quaternion#################################
		Eigen::Matrix<double, 3, 1> v;
		Eigen::Matrix<double, 3, 1> v_sum;

		if (count==1){
			anchor=T_S_I;
			ROS_INFO(" ");
			ROS_INFO("ANCHOR SET");
			ROS_INFO(" ");
		} else{

			v=box_minus(T_S_I,anchor);
			v_sum+=v;
			R_S_I_avg=box_plus(anchor,v_sum/count);
			ROS_INFO_STREAM(R_S_I_avg.vector());
		}

		//#################################################################################

		sum_sensor_ircamera=sum_sensor_ircamera+T_S_I.getTransformationMatrix();

		//Print Euler and Transformation Matrix of current frame
		//ROS_INFO("Euler Angles:");
		//ROS_INFO_STREAM(T_S_I.getRotationMatrix().eulerAngles(2,0,2)*180/M_PI);
		ROS_INFO("Transformation Matrix:");
		ROS_INFO_STREAM(T_S_I.getTransformationMatrix());

		sensor_ircamera=T_S_I.getTransformationMatrix();
		euler_angles=T_S_I.getRotationMatrix().eulerAngles(2,0,2);
		euler_angles*=180/M_PI;
		ROS_INFO("Euler Angles:");
		ROS_INFO_STREAM(euler_angles);

		//store data for statistics
		v_dx.push_back(sensor_ircamera(0,3)*1000);
		v_dy.push_back(sensor_ircamera(1,3)*1000);
		v_dz.push_back(sensor_ircamera(2,3)*1000);
		v_rX.push_back(euler_angles(0));
		v_rY.push_back(euler_angles(1));
		v_rZ.push_back(euler_angles(2));



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
