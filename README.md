# vicon_depthcam

Tools for using a depth-camera in combination with Vicon tracking.

### This library has three ROS nodes

1) vicon_checker_calib - This node uses a checkerboard with additional Vicon markers to find the transformation between a depth camera output and the coordinate frame given by its Vicon markers.

2) pointcloud_to_global - Combines a point cloud with a Vicon transform to give the points in the Vicon frame.

3) image_converter - Converts images to 8-bit (needed for calibrating 16-bit ir images produced by many depth cams)

These functions are based off of work performed by Philipp Egger (See the report.pdf for further details)
