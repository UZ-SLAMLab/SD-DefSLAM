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

#ifndef WARPDATABASE_H
#define WARPDATABASE_H

#include "KeyFrame.h"
#include "MapPoint.h"
#include "diffProp.h"

#include <memory>
#include <mutex>

namespace defSLAM {
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::MapPoint;
typedef std::vector<std::shared_ptr<DiffProp>> kr2krdata;

class DiffProp;
class WarpDatabase {
public:
  virtual ~WarpDatabase() = default;
  virtual void add(KeyFrame *kf) = 0;

  virtual void erase(KeyFrame *kf) = 0;

  virtual void clear() = 0;

  virtual bool getDiffPropVector(KeyFrame *, KeyFrame *, uint,
                                 DiffProp &DiffReq) = 0;

  bool process;

  std::map<MapPoint *, kr2krdata> &getDiffDatabase() { return mapPointsDB_; }

  std::map<MapPoint *, bool> &getToProccess() { return newInformation_; }

protected:
  std::map<std::pair<KeyFrame *, KeyFrame *>,
           std::vector<std::shared_ptr<DiffProp>>>
      DiffDatabase_;

  std::map<MapPoint *, bool> newInformation_;

  std::map<MapPoint *, kr2krdata> mapPointsDB_;
};

} // namespace ORB_SLAM

#endif
