#include <ros/ros.h>
#include <image_transport/image_transport.h>

/*
 *This node converts the pseudo 16bit-IR_Image to a 8-bit picture in order to be readable for the checkerboard-detection node.
 */

class Forward {
public:

	void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

		sensor_msgs::Image msgcopy;
		msgcopy = *msg;
		int factor = msgcopy.step / msgcopy.width;
		unsigned long length = msgcopy.width * msgcopy.height;

		//copy data to first part fo the array
		for (int i = 1; i < length; i++) {
			msgcopy.data[i] = msgcopy.data[2 * i];
		}
		//delete second part of the array
		msgcopy.data.resize(length);
		msgcopy.encoding = "mono8";
		msgcopy.step = msgcopy.width;

		pub_.publish(msgcopy);
		ROS_INFO("Published converted image.");
	}

	Forward() {
		image_transport::ImageTransport it(nh);
		pub_ = it.advertise("camera/ir/image_rect_conv", 1);
		sub_ = it.subscribe("camera/ir/image_rect_ir", 1,
				&Forward::imageCallback, this);
	}

private:
	ros::NodeHandle nh;
	image_transport::Publisher pub_;
	image_transport::Subscriber sub_;
	//image_transport::ImageTransport it(nh);
};

int main(int argc, char **argv) {

	ros::init(argc, argv, "forwarder");

	Forward ForwardObj;

	ros::spin();

	return 0;
}

