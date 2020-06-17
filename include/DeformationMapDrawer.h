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
#ifndef DEFMAPDRAWER_H
#define DEFMAPDRAWER_H

#include "MapDrawer.h"
#include "MeshDrawer.h"

#include <pangolin/pangolin.h>

#include "Tracking.h"
//#include "MeshDrawer.h"
#include <mutex>
#define MAX_NODES 300000
#define MAX_FACETS 100000

namespace ORB_SLAM2 {
class MapDrawer;

}

namespace defSLAM{
class Node;
class Edge;
class Facet;
class LaplacianMesh;
using ORB_SLAM2::MapDrawer;

class DeformationMapDrawer : public MapDrawer {
public:
  DeformationMapDrawer(Map *, const string &);
  void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
  void DrawTemplate(uint o);
  void DrawTemplateAtRest(uint o);
  void DrawTemplatehist();

  void DrawRefPoints();

  void DrawPointsAtRest();
  void reset();
  void UpdateTemplate();
  void UpdateTemplateAtRest();

  void UpdatePointsAtRest(KeyFrame *pFrame, float s);

  void ShowCurvature(bool);

  void ShowTexture(bool);

  std::map<KeyFrame *, std::vector<Facet *>> keyframeToFacet;

  std::map<KeyFrame *, GLuint> KeyFrameIndex;

protected:
  // void UpdatePoints();
  MeshDrawer *MeshDrawers;
  std::map<KeyFrame *, unique_ptr<MeshDrawer>> MeshDrawershist;

protected:
  float mTemplateLineWidth;

  bool Curvature;

  uint NumberNodes;
  uint NumberFacets;

  std::mutex mcurvature;

  bool Texture;

  std::mutex mTexture;
  std::mutex mTemplate;
  std::mutex mTemplatehist;

  std::mutex mPointsar;
};

} // namespace ORB_SLAM2

#endif // DEFORMATION MAPDRAWER_H
