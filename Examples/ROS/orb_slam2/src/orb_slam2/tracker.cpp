#include "tracker.h"

using namespace ORB_SLAM2;

Tracker::Tracker(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
                 KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor)
  : Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pMap, pKFDB, strSettingPath, sensor)
{

}
