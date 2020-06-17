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

#ifndef DEFOPTIMIZER_H
#define DEFOPTIMIZER_H

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"

#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
// Fwr declaration
namespace ORB_SLAM2 {
class Frame;
class Map;
}
namespace defSLAM {
using ORB_SLAM2::Frame;
using ORB_SLAM2::Map;

namespace Optimizer {
int PoseOptimization(Frame *pFrame, ofstream &myfile);
// Shape-from-template
int DefPoseOptimization(Frame *pFrame, Map *mMap, double RegLap = 5000,
                        double RegInex = 5000, double RegTemp = 0,
                        uint NeighboursLayers = 1);

int DefPoseOptimization(const std::vector<std::vector<double>> &matches,
                        Frame *pframe, Map *mMap, std::vector<bool> &outlier,
                        double RegLap = 5000, double RegInex = 5000,
                        double RegTemp = 0);

bool OptimizeHorn(std::vector<std::vector<float>> &pts1,
                  std::vector<std::vector<float>> &pts2, g2o::Sim3 &g2oS12,
                  double chi, double huber);
}

} // namespace DefSLAM

#endif // OPTIMIZER_H
