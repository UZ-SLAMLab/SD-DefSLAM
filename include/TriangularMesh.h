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

#ifndef TRIANGULARMESH_H
#define TRIANGULARMESH_H
#include "opencv2/opencv.hpp"
#include <Facet.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <Optimizer.h>
#include <vtkCellArray.h>
#include <vtkDelaunay2D.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "Template.h"
#include "Timer.h"
#include <Converter.h>

namespace defSLAM {

class Template;
class MapPoint;
class KeyFrame;
class Frame;

class TriangularMesh : public Template {
public:
  TriangularMesh() = default;

  TriangularMesh(std::set<MapPoint *> &mspMapPoints, Map *);

  TriangularMesh(std::vector<std::vector<double>> &,
                 std::vector<std::vector<int>> &, Map *map);

  TriangularMesh(std::set<MapPoint *> &mspMapPoints, Map *map, KeyFrame *kf);

  virtual ~TriangularMesh();

  std::map<KeyFrame *, std::vector<Facet *>> keyframeToFacet;

public:
  virtual void generateCoordinates(MapPoint *, cv::Mat &Pos,
                                   cv::Mat &CamPosition, cv::KeyPoint &kp);
  void GetFacetTexture();
  void GetFacetTexture(KeyFrame *KF);

private:
  void PoissonTriangulation(std::vector<std::vector<float>> &,
                            std::vector<std::vector<int>> &);

  vtkSmartPointer<vtkPolyData>
  DelaunayTriangulation(std::vector<std::vector<float>> &vertexC);

  void SetNodes(std::vector<std::vector<float>> &,
                std::vector<std::vector<int>> &);

  void SetNodes(vtkSmartPointer<vtkPolyData> mesh,
                std::vector<std::vector<float>> &vertexW);

  void CalculateFeaturesCoordinates();

  bool pointInTriangle(const Eigen::Vector3f &query_point,
                       const Eigen::Vector3f &triangle_vertex_0,
                       const Eigen::Vector3f &triangle_vertex_1,
                       const Eigen::Vector3f &triangle_vertex_2,
                       Eigen::Vector3f &barycentric);

  void deletepointsout();

  void DiscardFaces();
};
}
#endif
