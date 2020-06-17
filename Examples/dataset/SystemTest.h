/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEMTEST_H
#define SYSTEMTEST_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "System.h"
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"


namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class SystemTest: public System
{


public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    SystemTest(const string &strVocFile, const string &strSettingsFile,  std::vector<std::vector<double>> &vertex, std::vector<std::vector<int>> &indexes, const bool bUseViewer = true);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, std::vector<std::vector<double> > &matches, const double &timestamp, const cv::Mat &TcwO);
    virtual void setMatches3D(std::vector<std::vector<double>> &);

    std::vector<std::vector<double>> get_Matches3D(){
        return Matches3D;
    }
    std::vector<bool> get_Outliers(){
        return outliers;
    }

protected:
    std::vector<std::vector<double>> Matches3D;
    std::vector<bool> outliers;

};

}// namespace ORB_SLAM

#endif // SYSTEM_H
