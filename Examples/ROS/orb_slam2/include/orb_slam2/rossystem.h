#ifndef ROSSYSTEM_H
#define ROSSYSTEM_H

#include "System.h"
#include "tracker.h"
#include "slamviz.h"

namespace ORB_SLAM2 {

class ROSSystem : public System
{
public:
    ROSSystem(ros::NodeHandle &node, const string &strVocFile, const string &strSettingsFile, const eSensor sensor, bool pangolin=false);
    void updateViz() { viz->publish(); }
		decltype(auto) getCurrentRotation() {
			return tracker->mCurrentFrame.GetRotationInverse().t();
		}
		decltype(auto) getCurrentTranslation() {
			return tracker->mCurrentFrame.GetCameraCenter();
		}
// private:
    SLAMViz *viz;
    Tracker *tracker;
};

}

#endif //ROSSYSTEM_H
