#include "rossystem.h"

using namespace ORB_SLAM2;

ROSSystem::ROSSystem(ros::NodeHandle &node, const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor, bool pangolin)
  : System(strVocFile, strSettingsFile, sensor, pangolin)
{
  viz = new SLAMViz(node, *this, mpMap, "orbslam_viz");
  tracker = new Tracker(this, mpVocabulary, viz, mpMapDrawer, mpMap, mpKeyFrameDatabase, strSettingsFile, sensor);
  this->mpTracker = tracker;
  this->mpLocalMapper->SetTracker(tracker);
  this->mpLoopCloser->SetTracker(tracker);
  tracker->SetLocalMapper(this->mpLocalMapper);
  tracker->SetLoopClosing(this->mpLoopCloser);
}
