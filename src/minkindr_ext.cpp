/*
 * adds the missing box-plus and box-minus to the minkindr library
 */

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"

Eigen::Matrix<double, 3, 1> box_minus(kindr::minimal::QuatTransformation T1, kindr::minimal::QuatTransformation T2){

	//std::vector<double> v (3);
	//kindr::minimal::RotationQuaternionTemplate::Vector3 v;
	Eigen::Matrix<double, 3, 1> v;
	v=(T1.getRotation()*T2.getRotation().inverse()).log();


return v;

}


kindr::minimal::RotationQuaternion box_plus(kindr::minimal::QuatTransformation T, Eigen::Matrix<double, 3, 1> v_avg){
	ROS_INFO("Box_Plus is called");
	kindr::minimal::RotationQuaternion T_avg;
	 T_avg=(kindr::minimal::RotationQuaternion::exp(v_avg)*T.getRotation()).normalize();

	 return T_avg;

}
