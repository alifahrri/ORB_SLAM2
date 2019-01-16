#ifndef SLAMVIZ_H
#define SLAMVIZ_H

// orb slam stuff
#include "Tracking.h"
#include "System.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "FrameDrawer.h"

// ros stuff
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2/buffer_core.h>
#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <vector>

namespace ORB_SLAM2
{

class ROSSystem;

class SLAMViz : public FrameDrawer
{
public :
	SLAMViz(ros::NodeHandle &node, ROSSystem &system, Map *map, std::string topic_name);
	// publish array marker
	void publish();
	// publish current image frame with keypoints
	void publishCurrentFrame();
	// publish transform
	void publishStaticTF();

private :
	void drawCurrentCamera();
	void drawMapPoints();
	void drawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

private :
	ros::Publisher pub;
	// tf::TransformBroadcaster br;
	visualization_msgs::MarkerArray marker_array;

	// image transporeter for publishing cv mat
	image_transport::ImageTransport transporter;
	image_transport::Publisher cv_pub;

	// orb slam ptr & reference
	// System &system;
	ROSSystem &system;
	Map *map = nullptr;
	Tracking *tracking = nullptr;

	// rviz marker settings
	size_t seq = 0;
	std::string topic_name;
	double key_frame_size = 0.05;
	double key_frame_line_width = 0.05;
	double graph_line_width = 0.05;
	double point_scale = 0.025;
	double camera_size = 0.05;
	double camera_line_width = 0.05;

	std::string world_tf_name = std::string("world");

	// tf broadcaster for key frames
	tf2_ros::TransformBroadcaster br;

	// to transform orbslam frame to ros (rviz) frame :
	tf2_ros::StaticTransformBroadcaster static_br;
	std::string key_frame_tf = std::string("key_frame_tf");
};

}

#endif // RVIZSLAM_H
