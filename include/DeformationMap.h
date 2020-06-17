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
#ifndef DEFMAP_H
#define DEFMAP_H

#include "Map.h"

#include "DeformationMapPoint.h"
#include "Template.h"
#include <set>

#include <mutex>

namespace ORB_SLAM2 {
  class Map;
  class Template;
  class KeyFrame;
  class MapPoint;
}

namespace defSLAM {
using ORB_SLAM2::Map;
using ORB_SLAM2::KeyFrame;

class DeformationMap : public Map {
public:
  DeformationMap();

  DeformationMap(std::vector<std::vector<double>> &,
                 std::vector<std::vector<int>> &);

  virtual void CreateTemplate();

  virtual void CreateTemplate(KeyFrame *Kf);

  virtual void clear();

  Template *GetTemplate();

  void clearTemplate();

  bool ThereIsATemplate;

  std::mutex MutexUpdating;
  KeyFrame *getLastKFTemplate();
  void setLastKFTemplate(KeyFrame *);

protected:
  Template *Mesh;
  KeyFrame *lastKfTemplate_;
  std::mutex MutexUsingMap;
  std::mutex MutexKf;
};

} // namespace ORB_SLAM

#endif // DEFORMABLE MAP_H
