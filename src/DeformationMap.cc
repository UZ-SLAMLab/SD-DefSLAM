#include "DeformationMap.h"
#include "TemplateGenerator.h"

#include <mutex>

namespace defSLAM {

DeformationMap::DeformationMap()
    : Map(), ThereIsATemplate(false), Mesh(static_cast<Template *>(nullptr)) {}

DeformationMap::DeformationMap(std::vector<std::vector<double>> &vertex,
                               std::vector<std::vector<int>> &index)
    : Map(), Mesh(static_cast<Template *>(nullptr)) {

  Mesh = TemplateGenerator::ChargeLaplacianMesh(vertex, index, this);
}

void DeformationMap::CreateTemplate() {
  // it creates a template with the current map points
  Mesh = TemplateGenerator::LaplacianMeshcreate(mspMapPoints, this);
  //    std::cout << "New Template created" << std::endl;
  ThereIsATemplate = true;
}

void DeformationMap::CreateTemplate(KeyFrame *Kf) {
  // It creates a template with the current mapped points
  Mesh = TemplateGenerator::LaplacianMeshCreate(mspMapPoints, this, Kf);

  ThereIsATemplate = true;
}

Template *DeformationMap::GetTemplate() { return Mesh; }

void DeformationMap::clearTemplate() {
  ThereIsATemplate = false;
  if (Mesh) {
    delete Mesh;
    Mesh = static_cast<Template *>(nullptr);
  }
  for (set<MapPoint *>::iterator sit = mspMapPoints.begin(),
                                 send = mspMapPoints.end();
       sit != send; sit++) {
    static_cast<DeformationMapPoint *>(*sit)->SetFacet(
        static_cast<Facet *>(nullptr));
  }
}

void DeformationMap::clear() {
  this->clearTemplate();
  for (set<MapPoint *>::iterator sit = mspMapPoints.begin(),
                                 send = mspMapPoints.end();
       sit != send; sit++)
    delete *sit;

  for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(),
                                 send = mspKeyFrames.end();
       sit != send; sit++)
    delete *sit;

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid = 0;
  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
}

KeyFrame *DeformationMap::getLastKFTemplate() {
  std::unique_lock<std::mutex> lck(MutexKf);
  return lastKfTemplate_;
}

void DeformationMap::setLastKFTemplate(KeyFrame *kf) {
  std::unique_lock<std::mutex> lck(MutexKf);
  lastKfTemplate_ = kf;
}
} // namespace ORB_SLAM2
