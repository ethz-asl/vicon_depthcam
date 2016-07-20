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
#include <Eigen/Dense>
#include "find_closest_message.cpp"
#include "calib_data.cpp"

/*
 * This Node converts the pointcloud from the IR_Camera's Frame to the Global/Vicon Frame
 */


class PC_Conversion
{
public:

	void callback_vicon_sensor (const geometry_msgs::TransformStamped::ConstPtr& input)
	{
		sensor_vicon_msgs.push_front(*input);

	}

	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

		count++;
		if (((count > (factor - 1)) && !sensor_vicon_msgs.empty())
				|| set_freq) {
			count = 0;

			//find closest vicon msg
			int index=nns(input->header.stamp.toSec(),sensor_vicon_msgs);

			sensor_msgs::PointCloud2 output;

			//QuatTransformation from Vicon to Sensor
			kindr::minimal::QuatTransformation T_W_S;
			tf::transformMsgToKindr(sensor_vicon_msgs[index].transform, &T_W_S);

			//Overall Transform World to IR_Camera
			kindr::minimal::QuatTransformation T_W_I = T_W_S * T_S_I;

			static tf::TransformBroadcaster br;
			tf::Transform world_ircamera;
			tf::Transform ircamera_world;
			tf::transformKindrToTF(T_W_I, &world_ircamera);
			br.sendTransform(
					tf::StampedTransform(world_ircamera, ros::Time::now(),
							"world", "ir_direct"));
			ircamera_world = world_ircamera.inverse();

			// Tranbsform into global (vicon) frame
			pcl_ros::transformPointCloud("world", world_ircamera, *input,
					output);

			// Publish the data.
			pub_.publish(output);
		}

	}

	PC_Conversion()
		{
		    transformation=Stored_Transformation();
			pub_=n_.advertise<sensor_msgs::PointCloud2> ("points_global", 1);
			sub_pointcloud=n_.subscribe("camera/depth/points", 1, &PC_Conversion::cloud_cb, this);
	        sub_vicon_sensor=n_.subscribe("vrpn_client_kinect/raw_transform", 1, &PC_Conversion::callback_vicon_sensor, this);
	        T_S_I=kindr::minimal::QuatTransformation(transformation.getQuaternion(), transformation.getTranslation());
			count=0;
			target_rate=2;
			factor=30/target_rate;
			set_freq=1;
		}


private:

	Stored_Transformation transformation;

	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_pointcloud;
	ros::Subscriber sub_vicon_sensor;

	int count, target_rate, factor, set_freq;


	std::deque<geometry_msgs::TransformStamped> sensor_vicon_msgs;


	Eigen::Matrix<double, 3, 3> temp_rot_matrix;
	Eigen::Matrix<double, 3, 1> temp_translation;
	Eigen::Quaternion <double> temp_quat;
	//kindr::minimal::RotationQuaternion temp_rot_quat;
	//Eigen::Matrix<double, 4, 4> sensor_ircamera_transMatrix;
	kindr::minimal::QuatTransformation T_S_I;
};





int main(int argc, char **argv)
{

  ros::init(argc, argv, "pointcloud_to_global");

  PC_Conversion pc_conv;

  ros::spin();

  return 0;
}

