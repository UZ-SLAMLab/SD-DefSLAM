#include <Template.h>

#include <DeformationMapPoint.h>

namespace defSLAM {

Template::Template(std::set<MapPoint *> &mspMapPoints, Map *map, KeyFrame *kf)
    : mMap(map), numVertices(0), kf(kf) {
  for (std::set<MapPoint *>::iterator it = mspMapPoints.begin();
       it != mspMapPoints.end(); it++) {
    if (*it) {
      if (!(*it)->isBad()) {
        static_cast<DeformationMapPoint *>(*it)->AssignedTemplate(this);
        this->AddMapPoint(*it);
      }
    }
  }
  mspEdgesTemplate.clear();
}

Template::Template(Map *map) : mMap(map) {}

Template::~Template() {
  std::unique_lock<std::mutex> lc(MutexUpdating);
  texture.release();
  mMap = nullptr;
  for (std::set<MapPoint *>::iterator itmp = mspPointTemplate.begin();
       itmp != mspPointTemplate.end(); itmp++) {
    if (*itmp)
      if (!(*itmp)->isBad())
        static_cast<DeformationMapPoint *>(*itmp)->RemoveTemplate();
  }
  mspPointTemplate.clear();
  {
    std::unique_lock<std::mutex> a(mMutexEdges);

    for (std::set<Edge *>::iterator ite = mspEdgesTemplate.begin();
         ite != mspEdgesTemplate.end(); ite++) {
      delete (*ite);
    }
    mspEdgesTemplate.clear();
  }
  {
    std::unique_lock<std::mutex> a(mMutexFaces);

    for (std::set<Facet *>::iterator itf = mspFacets.begin();
         itf != mspFacets.end(); itf++) {
      delete (*itf);
    }
    mspFacets.clear();
  }

  {
    std::unique_lock<std::mutex> a(mMutexNodes);
    for (std::set<Node *>::iterator itn = mspNodeTemplate.begin();
         itn != mspNodeTemplate.end(); itn++) {
      delete (*itn);
    }
    mspNodeTemplate.clear();
  }

  Facet::TexturesDataset.clear();
}

cv::Mat Template::getTexture() { return this->texture.clone(); }

void Template::EraseMapPoint(MapPoint *pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspPointTemplate.erase(pMP);
}

void Template::AddMapPoint(MapPoint *pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspPointTemplate.insert(pMP);
}

void Template::AddNode(Node *pN) {
  unique_lock<mutex> lock(mMutexNodes);
  mspNodeTemplate.insert(pN);
}

void Template::EraseNode(Node *pN) {
  unique_lock<mutex> lock(mMutexNodes);
  mspNodeTemplate.erase(pN);
}

void Template::AddFacet(Facet *pN) {
  unique_lock<mutex> lock(mMutexFaces);
  mspFacets.insert(pN);
}

void Template::EraseFacet(Facet *pN) {
  unique_lock<mutex> lock(mMutexFaces);
  mspFacets.erase(pN);
}

void Template::AddEdge(Edge *pN) {
  unique_lock<mutex> lock(mMutexEdges);
  mspEdgesTemplate.insert(pN);
}

void Template::EraseEdge(Edge *pN) {
  unique_lock<mutex> lock(mMutexEdges);
  mspEdgesTemplate.erase(pN);
}

double Template::get_scale() {
  // std::unique_lock<std::mutex> lc(MutexUpdating);
  unique_lock<mutex> lock(mMutexEdges);

  vector<double> dists;
  for (std::set<Edge *>::iterator ite = mspEdgesTemplate.begin();
       ite != mspEdgesTemplate.end(); ite++) {
    dists.push_back((*ite)->get_dist());
  }
  std::sort(dists.begin(), dists.end());
  if (dists.size() > 0)
    return dists[dists.size() / 2];
  else {
    return 0.10;
  }
}
const std::set<MapPoint *> Template::get_Points() {
  return this->mspPointTemplate;
}

const std::set<Node *> Template::get_Nodes() { return this->mspNodeTemplate; }

const std::set<Edge *> Template::get_Edges() { return this->mspEdgesTemplate; }

const std::set<Facet *> Template::get_Facets() { return mspFacets; }
void Template::restart() {
  for (std::set<Node *>::iterator it = mspNodeTemplate.begin();
       it != mspNodeTemplate.end(); it++) {
    (*it)->reset();
  }
}

bool Template::UpdateFinished() {
  std::unique_lock<std::mutex> Mutex(MutexUpdating);
  bool Continue = !Updating;
  return Continue;
}

void Template::StartUpdate() {
  std::unique_lock<std::mutex> Mutex(MutexUpdating);
  Updating = true;
}

void Template::FinishUpdate() {
  std::unique_lock<std::mutex> Mutex(MutexUpdating);
  Updating = false;
}

bool Template::UseMap() {
  std::unique_lock<std::mutex> Mutex(MutexUsingMap);
  bool Continue = !UsingMap;
  return Continue;
}

void Template::StartUsing() {
  std::unique_lock<std::mutex> Mutex(MutexUsingMap);
  UsingMap = true;
}

void Template::FinishUsing() {
  std::unique_lock<std::mutex> Mutex(MutexUsingMap);
  UsingMap = false;
}
}
