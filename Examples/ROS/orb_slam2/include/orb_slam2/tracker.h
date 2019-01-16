#ifndef TRACKER_H
#define TRACKER_H

#include "Map.h"
#include "Tracking.h"
#include "FrameDrawer.h"

namespace ORB_SLAM2 {

class Tracker : public Tracking {
public:
  Tracker(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
          KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
  void setFrameDrawer(FrameDrawer *fd) {
    this->mpFrameDrawer = fd;
  }
};

}

#endif // TRACKER_H
