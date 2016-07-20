/*
 *      This node uses the PCL library to calculate the orientation of the main plane in the Pointcloud.
 *      http://pointclouds.org/
 *      http://wiki.ros.org/pcl/
 */

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/transform_broadcaster.h>


ros::Publisher pub;

int count=0;
double sum_rX=0.0, sum_rY=0.0, sum_rZ=0.0;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	count++;
	static tf::TransformBroadcaster br;
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud.makeShared ());
    seg.segment (inliers, coefficients);

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish (ros_coefficients);

    //publish quaternion in tf
    std::vector<float> q(4);
    q=coefficients.values;
    tf::Quaternion quat(q[0],q[1],q[2],q[3]);
    double rZ, rY, rX;
    tf::Matrix3x3(quat).getEulerZYX(rZ, rY, rX);
    //tf::Matrix3x3(quat).
    rZ=rZ*180/3.14159;
    rY=rY*180/3.14159;
    rX=rX*180/3.14159;
    sum_rZ+=rZ;
    sum_rY+=rY;
    sum_rX+=rX;
    std::cout << "rZ: " << rZ << ", rY: " << rY << ", rX: " << rX << std::endl;
    //std::cout << "Average:  rZ: " << sum_rZ/count << ", rY: " << sum_rY/count << ", rX: " << sum_rX/count << std::endl;
    tf::Vector3 pos(0.0,0.0,0.0);
    tf::Transform transform(quat,pos);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "plane_normal"));


}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "test_extrinsics");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/points_global", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("plane_coefficients", 1);

  // Spin
  ros::spin ();
}
