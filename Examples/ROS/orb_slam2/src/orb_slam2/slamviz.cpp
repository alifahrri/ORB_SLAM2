#include "slamviz.h"
#include "rossystem.h"
#include <sensor_msgs/image_encodings.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

// for filename info
#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

using namespace ORB_SLAM2;

SLAMViz::SLAMViz(ros::NodeHandle &node, ROSSystem &system, Map *map, std::string topic_name)
	: system(system)
	, FrameDrawer(map)
	, map(map)
	  // , tracking(tracking)
	, topic_name(topic_name)
	, transporter(image_transport::ImageTransport(node))
{
	pub = node.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
	cv_pub = transporter.advertise("/orb_slam2/current_frame",1);
	this->publishStaticTF();
}

void SLAMViz::publish()
{
#ifdef TRACE
	ROS_WARN(__PRETTY_FUNCTION__);
	ROS_WARN("line : %d", __LINE__);
#endif
	this->drawMapPoints();
#ifdef TRACE
	ROS_WARN("line : %d", __LINE__);
#endif
	this->drawKeyFrames(true, false);
	pub.publish(marker_array);
	seq++;
#ifdef TRACE
	ROS_WARN("line : %d", __LINE__);
#endif
	marker_array.markers.clear();
#ifdef TRACE
	ROS_WARN("%s %s",__PRETTY_FUNCTION__, "done");
#endif
}

void SLAMViz::publishCurrentFrame()
{
	auto frame = this->DrawFrame();
	if(frame.empty()) return;
	cv_bridge::CvImage cv_ptr;
	cv_ptr.image = frame;
	cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
	cv_ptr.header.stamp = ros::Time::now();
	cv_ptr.header.seq = seq;
	cv_pub.publish(cv_ptr.toImageMsg());
}

void SLAMViz::publishStaticTF()
{
	geometry_msgs::TransformStamped tr;
	tf2::Quaternion q;
	
	// create transform from world frame to orbslam frame
	tr.header.frame_id = world_tf_name;
	tr.header.stamp = ros::Time::now();
	tr.child_frame_id = key_frame_tf;
	q.setRPY(M_PI_2,0.0,0.0);
	tr.transform.rotation.x = q.x();
	tr.transform.rotation.y = q.y();
	tr.transform.rotation.z = q.z();
	tr.transform.rotation.w = q.w();
	static_br.sendTransform(tr);
	
	// create transform from current frame to base link (for point cloud)
	tr.header.frame_id = "current_frame";
	tr.header.stamp = ros::Time::now();
	tr.child_frame_id = "base_link";
	q.setRPY(M_PI_2,0.0,M_PI_2);
	tr.transform.rotation.x = q.x();
	tr.transform.rotation.y = q.y();
	tr.transform.rotation.z = q.z();
	tr.transform.rotation.w = q.w();
	static_br.sendTransform(tr);
}

void SLAMViz::drawMapPoints()
{
#ifdef TRACE
	ROS_WARN(__PRETTY_FUNCTION__);
#endif
	visualization_msgs::Marker marker;
	marker.ns = "map_points";
	marker.header.frame_id = key_frame_tf;
	marker.header.seq = seq;
	marker.header.stamp = ros::Time::now();
	marker.id = marker_array.markers.size();
	marker.type = visualization_msgs::Marker::POINTS;
	marker.scale.x = marker.scale.y = marker.scale.z = point_scale;

	std_msgs::ColorRGBA color;
	color.r = color.g = color.b = color.a = 1.0;

	if(!map) {
		ROS_ERROR("map is not valid!!!");
		return;
	}
#ifdef TRACE
	ROS_WARN("line : %d", __LINE__);
#endif
	if(!map->MapPointsInMap())
		return;

#ifdef TRACE
	ROS_WARN("line : %d", __LINE__);
#endif
	auto vpMPs = map->GetAllMapPoints();
	if(vpMPs.empty())
		return;

#ifdef TRACE
	ROS_WARN("line : %d", __LINE__);
#endif
	auto vpRefMPs = map->GetReferenceMapPoints();

#ifdef TRACE
	ROS_WARN("line : %d", __LINE__);
#endif
	std::set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
	for(size_t i=0, iend=vpMPs.size(); i<iend; i++) {
		if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
			continue;
#ifdef TRACE
		ROS_WARN("line : %d", __LINE__);
#endif
		auto pos = vpMPs[i]->GetWorldPos();
		geometry_msgs::Point p;
		p.x = pos.at<float>(0);
		p.y = pos.at<float>(1);
		p.z = pos.at<float>(2);
		marker.points.push_back(p);
		marker.colors.push_back(color);
	}

	color.r = color.g = 0.0;
#ifdef TRACE
	ROS_WARN("line : %d", __LINE__);
#endif
	for(auto sit = spRefMPs.begin(), send = spRefMPs.end(); sit!=send; sit++) {
		if((*sit)->isBad()) continue;
#ifdef TRACE
		ROS_WARN("line : %d", __LINE__);
#endif
		auto pos = (*sit)->GetWorldPos();
		geometry_msgs::Point p;
		p.x = pos.at<float>(0);
		p.y = pos.at<float>(1);
		p.z = pos.at<float>(2);
		marker.points.push_back(p);
		marker.colors.push_back(color);
	}

#ifdef TRACE
	ROS_WARN("line : %d", __LINE__);
#endif
	marker_array.markers.push_back(marker);
#ifdef TRACE
	ROS_WARN("%s %s",__PRETTY_FUNCTION__, "done");
#endif
}

void SLAMViz::drawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
	const auto h = key_frame_size * 0.75;
	const auto z = key_frame_size * 0.6;
	auto vpKFs = map->GetAllKeyFrames();
	auto now = ros::Time::now();

	// deduction
	using rot_t = decltype(vpKFs[0]->GetRotation());
	using trans_t = decltype(vpKFs[0]->GetTranslation());

	if(bDrawKF) {
		// types
		rot_t rot;
		trans_t trans;
		for(size_t i=0; i<vpKFs.size(); i++) {

			// get keyframe
			auto pKF = vpKFs[i];

			decltype(auto) Twc = pKF->GetPoseInverse();
			auto twc = cv::Mat(Twc);
			// transform.setFromOpenGLMatrix(cv::Mat(Twc).ptr<tfScalar>(0));

			// get cv rotation and trans
			// convert to tf quat and trans
			// set transform
			// rot = pKF->GetRotation();
			// trans = pKF->GetTranslation();
			rot = twc.rowRange(0,3).colRange(0,3);
			trans = twc.rowRange(0,3).col(3);

			// transform
			tf2::Matrix3x3 rot_mat(
			  rot.at<tfScalar>(0,0),rot.at<tfScalar>(0,1),rot.at<tfScalar>(0,2),
			  rot.at<tfScalar>(1,0),rot.at<tfScalar>(1,1),rot.at<tfScalar>(1,2),
			  rot.at<tfScalar>(2,0),rot.at<tfScalar>(2,1),rot.at<tfScalar>(2,2)
			);
			tf2::Vector3 origin(
			  trans.at<tfScalar>(0),
			  trans.at<tfScalar>(1),
			  trans.at<tfScalar>(2)
			);
			tf2::Quaternion quat;
			// rot_mat.getRotation(quat);
			tfScalar roll, pitch, yaw;
			rot_mat.getRPY(roll, pitch, yaw);
			quat.setRPY(roll, pitch, yaw);
			tf2::Transform transform(rot_mat, origin);

			// print
			std::stringstream ss;
			ss << "Twc: \n";
			ss << twc.at<float>(0,0) << ", " << twc.at<float>(0,1) << ", " << twc.at<float>(0,2) << ", " << twc.at<float>(0,3) << std::endl
			   << twc.at<float>(1,0) << ", " << twc.at<float>(1,1) << ", " << twc.at<float>(1,2) << ", " << twc.at<float>(1,3) << std::endl
			   << twc.at<float>(2,0) << ", " << twc.at<float>(2,1) << ", " << twc.at<float>(2,2) << ", " << twc.at<float>(2,3) << std::endl
			   << twc.at<float>(3,0) << ", " << twc.at<float>(3,1) << ", " << twc.at<float>(3,2) << ", " << twc.at<float>(3,3) << std::endl;
			ss << "rot: \n";
			ss << rot.at<float>(0,0) << ", " << rot.at<float>(0,1) << ", " << rot.at<float>(0,2) << ", " << std::endl
			   << rot.at<float>(1,0) << ", " << rot.at<float>(1,1) << ", " << rot.at<float>(1,2) << ", " << std::endl
			   << rot.at<float>(2,0) << ", " << rot.at<float>(2,1) << ", " << rot.at<float>(2,2) << std::endl;
			ss << "trans:\n";
			ss << trans.at<float>(0) << ", " << trans.at<float>(1) << ", " << trans.at<float>(2) << std::endl;

			// prepare tf message
			std::stringstream kf_str;
			kf_str << "kf_" << i;
			geometry_msgs::TransformStamped msg;
			msg.header.stamp = now;
			msg.header.frame_id = key_frame_tf;
			msg.child_frame_id = kf_str.str();

			// convert and send
			// set translation
			// msg.transform.translation = tf2::toMsg(origin);
			msg.transform.translation.x = trans.at<float>(0);
			msg.transform.translation.y = trans.at<float>(1);
			msg.transform.translation.z = trans.at<float>(2);
			// set rotation
			// msg.transform.rotation = tf2::toMsg(quat);
			// auto quat = transform.getRotation().normalize();
			msg.transform.rotation.x = quat.x();
			msg.transform.rotation.y = quat.y();
			msg.transform.rotation.z = quat.z();
			msg.transform.rotation.w = quat.w();
			// msg.transform = tf2::toMsg(transform);

			// print
			ss << "msg.translation :\n";
			ss << msg.transform.translation.x << ", " << msg.transform.translation.y << ", " << msg.transform.translation.z << std::endl;
			ss << "msg.rotation :\n";
			ss << msg.transform.rotation.x << ", " << msg.transform.rotation.y << ", " << msg.transform.rotation.z << ", " << msg.transform.rotation.w << std::endl;
			ROS_WARN("%s", ss.str().c_str());

			br.sendTransform(msg);
		}

		{ // draw current frame
			// get current transform in world frame
			rot = system.getCurrentRotation();
			trans = system.getCurrentTranslation();
			if(!rot.empty())
			{
				// rot = twc.rowRange(0,3).colRange(0,3);
				// trans = twc.rowRange(0,3).col(3);

				// transform
				tf2::Matrix3x3 rot_mat(
				  rot.at<tfScalar>(0,0),rot.at<tfScalar>(0,1),rot.at<tfScalar>(0,2),
				  rot.at<tfScalar>(1,0),rot.at<tfScalar>(1,1),rot.at<tfScalar>(1,2),
				  rot.at<tfScalar>(2,0),rot.at<tfScalar>(2,1),rot.at<tfScalar>(2,2)
				);
				tf2::Quaternion quat;
				// rot_mat.getRotation(quat);
				tfScalar roll, pitch, yaw;
				rot_mat.getRPY(roll, pitch, yaw);
				quat.setRPY(roll, pitch, yaw);

				// prepare tf message
				geometry_msgs::TransformStamped msg;
				msg.header.stamp = now;
				msg.header.frame_id = key_frame_tf;
				msg.child_frame_id = "current_frame";

				// convert and send
				// set translation
				msg.transform.translation.x = trans.at<float>(0);
				msg.transform.translation.y = trans.at<float>(1);
				msg.transform.translation.z = trans.at<float>(2);
				// set rotation
				msg.transform.rotation.x = quat.x();
				msg.transform.rotation.y = quat.y();
				msg.transform.rotation.z = quat.z();
				msg.transform.rotation.w = quat.w();
				// send transform
				br.sendTransform(msg);
			}
		}
	}

	if(bDrawGraph) {
		visualization_msgs::Marker marker;
		marker.ns = "graph";
		marker.header.frame_id = key_frame_tf;
		marker.header.seq = seq;
		marker.header.stamp = ros::Time::now();
		marker.id = marker_array.markers.size();
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.scale.x = marker.scale.y = marker.scale.z = point_scale;
		std_msgs::ColorRGBA color;
		color.r = color.g = color.b = color.a = 1.0;
		for(size_t i=0; i<vpKFs.size(); i++) {
			auto vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
			auto Ow = vpKFs[i]->GetCameraCenter();
			if(!vCovKFs.empty()) {
				for(auto vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++) {
					if((*vit)->mnId<vpKFs[i]->mnId) continue;
					auto Ow2 = (*vit)->GetCameraCenter();
					// insert line marker
					geometry_msgs::Point p, p2;
					p.x = Ow.at<float>(0);
					p2.x = Ow2.at<float>(0);
					p.y = Ow.at<float>(1);
					p2.y = Ow2.at<float>(1);
					p.z = Ow.at<float>(2);
					p2.z = Ow2.at<float>(2);
					marker.points.push_back(p);
					marker.points.push_back(p2);
					marker.colors.push_back(color);
					marker.colors.push_back(color);
				}
			}

			auto pParent = vpKFs[i]->GetParent();
			if(pParent) {
				auto Owp = pParent->GetCameraCenter();
				// insert line marker
				geometry_msgs::Point p, p2;
				p.x = Ow.at<float>(0);
				p2.x = Owp.at<float>(0);
				p.y = Ow.at<float>(1);
				p2.y = Owp.at<float>(1);
				p.z = Ow.at<float>(2);
				p2.z = Owp.at<float>(2);
				marker.points.push_back(p);
				marker.points.push_back(p2);
				marker.colors.push_back(color);
				marker.colors.push_back(color);
			}

			auto sLoopKFs = vpKFs[i]->GetLoopEdges();
			for(auto sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++) {
				if((*sit)->mnId<vpKFs[i]->mnId) continue;
				auto Owl = (*sit)->GetCameraCenter();
				// insert line marker
				geometry_msgs::Point p, p2;
				p.x = Ow.at<float>(0);
				p2.x = Owl.at<float>(0);
				p.y = Ow.at<float>(1);
				p2.y = Owl.at<float>(1);
				p.z = Ow.at<float>(2);
				p2.z = Owl.at<float>(2);
				marker.points.push_back(p);
				marker.points.push_back(p2);
				marker.colors.push_back(color);
				marker.colors.push_back(color);
			}
		}
		marker_array.markers.push_back(marker);
	}
}
