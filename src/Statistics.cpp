/*
 * offering a simple solution to print statistics of a vector
 */

#include <vector>
#include "ros/ros.h"



void print_stats(const std::vector<double> v) {
	int size = v.size();
	double sum = 0.0, temp = 0.0, mean = 0.0, var = 0.0, std_dev = 0.0;
	for (double a : v)
		sum += a;
	mean = sum / size;
	for (double a : v)
		temp += (mean - a) * (mean - a);
	var = temp / size;
	std_dev = sqrt(var);
	ROS_INFO_STREAM(
			"mean: " << mean << " var: " << var << " std_dev: " << std_dev);

}



