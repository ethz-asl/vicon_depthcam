#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class ImageConverter {
public:

  ImageConverter(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

  void image_callback(const sensor_msgs::ImageConstPtr& msg);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher image_pub_;
  ros::Subscriber image_sub_;
};
