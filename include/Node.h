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

#ifndef NODE_H
#define NODE_H

#include <Template.h>
#include <Edge.h>
#include <Facet.h>
#include <MapPoint.h>
#include <set>
#include <mutex>

namespace ORB_SLAM2{
class KeyFrame;
class Map;
class Frame;
class MapPoint;
}

namespace defSLAM {
class Template;
class Edge;
class Facet;

using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::Frame;
using ORB_SLAM2::MapPoint;


class Node : public MapPoint {
public:
  Node() = default;
  Node(double x, double y, double z, uint indx, Template *);
  ~Node();

  uint get_Index();
  void set_Index(uint i);
  void AssignEdge(Edge *);
  void AssignFacet(Facet *);
  void EraseFacet(Facet *);
  std::set<Edge *> Get_Edges();
  double Distanceto(Node *node);
  double x, y, z;
  double xO, yO, zO;
  float proju, projv;
  void deassigneFacet(Facet *facet);
  bool No_facets();
  void SetBadFlag();
  int PosVecDrawer;
  void deassignEdge(Edge *edge);
  std::set<Node *> GetNeighbours();
  std::set<Facet *> GetFacets();
  void setViewed();
  void setLocal();
  void setPropagated();
  void resetRole();
  void reset();
  void update();
  bool isViewed();
  bool isLocal();

  void setXYZ(double xx, double yy, double zz);
  void getXYZ(double &xx, double &yy, double &zz);
  void getInitialPose(double &xx, double &yy, double &zz);

  void setBoundary();
  bool isBoundary();

public:
  std::map<Node *, double> weights;
  std::map<Node *, std::pair<Node *, Node *>> NodesjJ_1J;

private:
  uint indx;
  Template *mTemplate;
  std::set<Edge *> mEdge;
  std::set<Facet *> mFacet;

  bool viewed, local;
  bool registred;
  enum eROLE { VIEWED = 0, LOCAL = 1, NONOBS = 3 };

  eROLE role;
  bool Boundary;
  std::mutex mMutexPos;
  std::mutex mMutexCond;
};

} // namespace ORB_SLAM

#endif // NODE_H
