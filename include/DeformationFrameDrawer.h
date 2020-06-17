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
#ifndef DEFFRAMEDRAWER_H
#define DEFFRAMEDRAWER_H

#include "FrameDrawer.h"

#include <mutex>

namespace ORB_SLAM2 {
class Map;
class Tracking;
class FrameDrawer;
}
namespace defSLAM{
using  ORB_SLAM2::FrameDrawer;
using  ORB_SLAM2::Tracking;
using  ORB_SLAM2::Map;

class DeformationFrameDrawer : public FrameDrawer {
public:
  DeformationFrameDrawer(Map *pMap);

  cv::Mat virtual DrawFrame();

  void virtual Update(Tracking *pTracker);

protected:
  void DrawTemplate(cv::Mat &im);

  cv::Mat mK, mTcw;

  vector<bool> vvbdeformable;
};

} // namespace ORB_SLAM

#endif // FRAMEDRAWER_H
