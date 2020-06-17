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

#ifndef FACET_H
#define FACET_H

#include <Template.h>
#include <KeyFrame.h>
#include <Edge.h>
#include <Node.h>
#include <MapPoint.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <mutex>

namespace ORB_SLAM2{
class KeyFrame;
class Map;
class Frame;
class MapPoint;
}
namespace defSLAM {
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;

class Template;
class Node;
class Edge;

class Facet {
public:
  Facet(uint, uint, uint, Template *);
  Facet(Node *, Node *, Node *, Template *);

  ~Facet();

  std::set<Node *> getNodes();
  bool NoObservations();
  void addMapPoint(MapPoint *);
  void EraseMapPoint(MapPoint *);

  std::set<defSLAM::Edge *> getEdges();
  std::set<MapPoint *> getMapPoints();
  void SetBadFlag();
  double getArea();
  void setKeyframes(std::set<KeyFrame *>);
  void getTextureCoordinates();
  void getTextureCoordinates(KeyFrame *KF);
  std::array<Node *, 3> getNodesArray();
  std::pair<KeyFrame *, std::map<Node *, cv::Point>> texture;
  double R, G, B;
  static std::map<KeyFrame *, std::set<Facet *>> TexturesDataset;

private:
  Template *mpTemplate;
  std::set<Node *> Nodes;
  std::array<Node *, 3> mNodes;

  std::set<Edge *> Edges;
  std::set<MapPoint *> mPoints;

  std::set<KeyFrame *> Keyframes;

  uint index;
  uint index_In_buffer;
  std::mutex mMuteFacet;
  std::mutex mMutexKeyframe;
};

} // namespace ORB_SLAM2

#endif // FACET_H
