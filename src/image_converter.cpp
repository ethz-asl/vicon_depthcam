#include "vicon_depthcam/image_converter.h"

ImageConverter::ImageConverter(const ros::NodeHandle& nh,
                               const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
  image_pub_ = private_nh_.advertise<sensor_msgs::Image>("image_mono8", 1);
  image_sub_ =
      nh_.subscribe("input_image", 1, &ImageConverter::image_callback, this);
}

void ImageConverter::image_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg,"mono16");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_ptr->image *= 255;
  cv_ptr = cv_bridge::cvtColor(cv_ptr, "mono8");
  image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");

  ros::NodeHandle nh, private_nh("~");

  ImageConverter image_converter(nh, private_nh);

  ros::spin();

  return 0;
}
