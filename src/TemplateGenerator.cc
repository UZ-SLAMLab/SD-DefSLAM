#include <TemplateGenerator.h>
#include <mutex>

#include "LaplacianMesh.h"

namespace defSLAM {
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::Map;
using ORB_SLAM2::KeyFrame;

class LaplacianMesh;

Template *
TemplateGenerator::LaplacianMeshcreate(std::set<MapPoint *> &mspMapPoints,
                                       Map *map) {
  LaplacianMesh *lapmesh = new LaplacianMesh(mspMapPoints, map);
  return lapmesh;
}

Template *
TemplateGenerator::ChargeLaplacianMesh(std::vector<std::vector<double>> &vertex,
                                       std::vector<std::vector<int>> &index,
                                       Map *map) {
  LaplacianMesh *lapmesh = new LaplacianMesh(vertex, index, map);

  return lapmesh;

  return (static_cast<LaplacianMesh *>(nullptr));
}

Template *
TemplateGenerator::LaplacianMeshCreate(std::set<MapPoint *> &mspMapPoints,
                                       Map *map, KeyFrame *kF) {
  LaplacianMesh *lapmesh = new LaplacianMesh(mspMapPoints, map, kF);

  return lapmesh;

  return (static_cast<LaplacianMesh *>(nullptr));
}

} // namespace ORB_SLAM2
