/**
* This file is part of DefSLAM.
*
* Copyright (C) 2017-2020 Jose Lamarca Peiro <jlamarca at unizar dot es> (University
*of Zaragoza)
* For more information see <https://github.com/unizar/DefSLAM>
*
* DefSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DefSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DefSLAM. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DEFLOCALMAPPING_H
#define DEFLOCALMAPPING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "WarpDatabase.h"

namespace ORB_SLAM2 {
class KeyFrame;
}
namespace defSLAM {
using ORB_SLAM2::Map;
using ORB_SLAM2::MapDrawer;
using ORB_SLAM2::KeyFrame;

class DeformationLocalMapping : public ORB_SLAM2::LocalMapping {
public:
  DeformationLocalMapping(Map *pMap, MapDrawer *mpDrawer,
                          const float bMonocular, const string &strSettingPath);

  ~DeformationLocalMapping();

  virtual void Run();

  bool UpdateTemplate();

  bool CreatingTemplate;

  virtual void ResetIfRequested();

  void CreatePoints();

  void insideTheLoop();

  bool needNewTemplate();

  KeyFrame *selectKeyframe();

protected:
  virtual void ProcessNewKeyFrame();
  // We create an estimation of the new points extrapolating the
  // current SfT solution.
  void CreateNewMapPoints();

  virtual void KeyFrameCulling();

  std::vector<KeyFrame *> LocalKeyframes;
  bool CreateTemplate;

  WarpDatabase *mpSchwarpDB;

  KeyFrame *KFtoTriangulate;

  bool NonRigidInitialisated;

  int pointsToTemplate_;
  double chiLimit_;
  double bendingReg_;
};

} // namespace ORB_SLAM2

#endif // LOCALMAPPING_H
