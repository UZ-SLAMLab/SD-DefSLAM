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

#ifndef SCHWARPDATABASE_H
#define SCHWARPDATABASE_H
#pragma once

#include <list>
#include <set>
#include <vector>

#include "WarpDatabase.h"

#include "KeyFrame.h"
#include "MapPoint.h"

#include <bbs_MAC.h>
#include <mutex>

namespace defSLAM {
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Frame;
using ORB_SLAM2::MapPoint;

class WarpDatabase;
class DiffProp;

class SchwarpDatabase : public WarpDatabase {
public:
  SchwarpDatabase(double reg) : reg_(reg) {}

  void add(KeyFrame *kf) override;

  void erase(KeyFrame *kf) override;

  void clear() override;

  bool getDiffPropVector(KeyFrame *, KeyFrame *, uint,
                         DiffProp &DiffReq) override;

protected:
  void CalculateSchwarps(
      KeyFrame *KF, KeyFrame *KF2,
      vector<pair<size_t, size_t>> &vMatchedIndices,
      double (&x)[_NumberOfControlPointsU * _NumberOfControlPointsV * 2],
      double reg, bool SaveWarp = true);

protected:
  std::set<KeyFrame *> mpkeyframes;
  double reg_;
};

} // namespace ORB_SLAM

#endif
