/*
 * This node offers the possibility to do any pointcloud preprocessing before fusing the cloud into the map representation (octomap/voxblox)
 * currently all it does is changing the frame-tag in order to convert it into the correct frame.
 */

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//a 30hz cloud topic is assumed to be given

ros::Publisher pub;
int count=0, factor=0;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	count++;
	if (count >(factor-1))
	{
	count=0;
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  output = *input;

  output.header.frame_id="ir_camera";

  //add any additional preprocessing here.

  // Publish the data.
  pub.publish (output);
	}
}

int
main (int argc, char** argv)
{
	int target_rate=5;
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_preprocessing");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("points_processed", 1);
  nh_private.param("frame_rate",target_rate,target_rate);
  factor=30/target_rate;

  // Spin
  ros::spin ();
}
