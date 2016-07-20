/*
 * function to search for the closest tf (by timestamp)
 */
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"

int nns(const double& time,
		const std::deque<geometry_msgs::TransformStamped>& msgs) {
	double diff = 9999.0;
	double new_diff = 10000.0;
	int iter = 0;
	//search closest checker_vicon_msg
	do {
		diff = new_diff;
		new_diff = fabs(msgs[iter].header.stamp.toSec() - time);
		//ROS_INFO("chessboard_vicon '%f'", checker_vicon_msgs[iter].header.stamp.toSec());
		//ROS_INFO("chessboard_detected '%f'", checker_detection_msg.header.stamp.toSec());
		//ROS_INFO("difference '%f'", new_diff);
		iter++;
	} while ((new_diff < diff && iter - 1 < msgs.size()));
	if (iter < 2) {
		iter = 2;
		ROS_INFO("Reference Message is more recent than the message list.");
		ROS_INFO("First Element chosen.");
	}

	return (iter - 2);
}

