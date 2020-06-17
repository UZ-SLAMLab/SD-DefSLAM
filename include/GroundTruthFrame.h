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

#ifndef GTFRAME_H
#define GTFRAME_H

#include "Frame.h"
#include "Map.h"
#include <thread>
#include <vector>

namespace ORB_SLAM2{
class Frame;
class Map;
}

namespace defSLAM {
using ORB_SLAM2::Frame;
using ORB_SLAM2::Map;
using ORB_SLAM2::ORBextractor;
using ORB_SLAM2::ORBVocabulary;

class GroundTruthFrame : public Frame {
public:
  GroundTruthFrame() = delete;

  // Copy constructor.
  GroundTruthFrame(const GroundTruthFrame &frame);

  // Initialization with stereo (just for ORBSLAM)
  GroundTruthFrame(const cv::Mat &imLeft, const cv::Mat &imRight,
                   const double &timeStamp,
                   ORB_SLAM2::ORBextractor *extractorLeft,
                   ORB_SLAM2::ORBextractor *extractorRight,
                   ORB_SLAM2::ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef,
                   const float &bf, const float &thDepth, const cv::Mat &ImRGB,
                   cv::Mat _mask = cv::Mat());
  // Initialization with monocular
  GroundTruthFrame(const cv::Mat &imGray, const double &timeStamp,
                   ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K,
                   cv::Mat &distCoef, const float &bf, const float &thDepth,
                   const cv::Mat &ImRGB, const cv::Mat &ImRight,
                   cv::Mat _mask = cv::Mat(), int action = 0);

  // Initialization with GT
  GroundTruthFrame(const cv::Mat &imGray, const double &timeStamp,
                   ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K,
                   cv::Mat &distCoef, const float &bf, const float &thDepth,
                   const cv::Mat &ImRGB, const cv::Mat &imDepth, bool isdepth,
                   cv::Mat _mask = cv::Mat());

public:
  double Estimate3DLocalMap(Map *map, const double &s);

  double Estimate3DScale(Map *map);

  std::vector<std::vector<float>> getPosMono();
  std::vector<std::vector<float>> getPosStereo();

private:
  std::vector<std::vector<float>> posMono_;
  std::vector<std::vector<float>> posStereo_;
  const int tempx_;
  const int tempy_;
  const int margin_;
  const int searchx_;
  const int searchy_;
  const double threshold_;
  bool isDepth;
  cv::Mat imDepth;

private:
  std::mutex mutexPoints;
};

} // namespace ORB_SLAM

#endif // GTFRAME_H
