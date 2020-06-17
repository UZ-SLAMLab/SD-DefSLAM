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

#ifndef TEMPLATE_H
#define TEMPLATE_H

#include "opencv2/core.hpp"
#include <Edge.h>
#include <Facet.h>
#include <MapPoint.h>
#include <Node.h>
#include <Surface.h>
#include <set>

namespace defSLAM {
class MapPoint;
class Node;
class Edge;
class Facet;
typedef std::vector<std::array<float, 3>> Point3DList;
typedef std::vector<std::array<float, 2>> Point2DList;
typedef std::vector<std::array<uint, 3>> Index3DList;

class Template {
public:
  Template(std::set<MapPoint *> &mspMapPoints, Map *map, KeyFrame *kf);

  Template(Map *map);

  virtual ~Template();

  void AddMapPoint(MapPoint *pMP);

  void EraseMapPoint(MapPoint *);

  void AddNode(Node *pN);

  void EraseNode(Node *);

  void AddFacet(Facet *);

  void EraseFacet(Facet *);

  void AddEdge(Edge *);

  void EraseEdge(Edge *);

  double get_scale();

  const std::set<MapPoint *> get_Points();

  const std::set<Node *> get_Nodes();

  const std::set<Facet *> get_Facets();

  const std::set<Edge *> get_Edges();

  virtual void generateCoordinates(MapPoint *, cv::Mat &, cv::Mat &,
                                   cv::KeyPoint &) = 0;

  bool UpdateFinished();

  void StartUpdate();

  void FinishUpdate();

  bool UseMap();

  void StartUsing();

  void FinishUsing();

  Point3DList getVertexArray() { return VerticesArray; }

  virtual void restart();

  cv::Mat getTexture();

public:
  std::vector<Node *> NodeDataSetArray;

  std::mutex MutexUpdating;

  KeyFrame *kf;

protected:
  Map *mMap;

  unsigned int numVertices;

  std::set<MapPoint *> mspPointTemplate;

  std::set<Node *> mspNodeTemplate;

  std::set<Edge *> mspEdgesTemplate;

  std::set<Facet *> mspFacets;

  Point3DList VerticesArray;
  Point3DList colour; // r, g, b, no alpha
  Index3DList faces;

  std::mutex mMutexMap;

  std::mutex mMutexNodes;

  std::mutex mMutexEdges;

  std::mutex mMutexFaces;

  std::mutex MutexUsingMap;

  cv::Mat texture;

  bool UsingMap;

  bool Updating;
};
}
#endif
