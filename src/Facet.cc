#include <Facet.h>
#include <mutex>

namespace defSLAM {
std::map<KeyFrame *, std::set<Facet *>> Facet::TexturesDataset;

Facet::Facet(uint v1, uint v2, uint v3, Template *temp) : mpTemplate(temp) {

  const std::set<Node *> &MapNodes = mpTemplate->get_Nodes();
  for (std::set<Node *>::iterator it = MapNodes.begin(); it != MapNodes.end();
       it++) {
    uint i = (*it)->get_Index();
    if ((i == v1) or (i == v2) or (i == v3)) {
      Nodes.insert(*it);
      (*it)->AssignFacet(this);
    }
  }

  std::set<Edge *> MapEdges = mpTemplate->get_Edges();
  for (std::set<Edge *>::iterator it = MapEdges.begin(); it != MapEdges.end();
       it++) {
    std::set<Node *> pairNodes = (*it)->get_nodes();

    bool FirstContained(false);
    bool SecondContained(false);

    for (std::set<Node *>::iterator ias = pairNodes.begin();
         ias != pairNodes.end(); ias++) {
      if ((Nodes.find(*ias) != Nodes.end()) and (FirstContained)) {
        SecondContained = true;
      } else if (Nodes.find(*ias) != Nodes.end()) {
        FirstContained = true;
      }
    }
    if (SecondContained) {
      Edges.insert(*it);
      (*it)->addFacet(this);
    }
  }
  texture.first = static_cast<KeyFrame *>(nullptr);
}

Facet::Facet(Node *v1, Node *v2, Node *v3, Template *temp) : mpTemplate(temp) {
  Nodes.insert(v1);
  (v1)->AssignFacet(this);
  Nodes.insert(v2);
  (v2)->AssignFacet(this);
  Nodes.insert(v3);
  (v3)->AssignFacet(this);
  mNodes[0] = v1;
  mNodes[1] = v2;
  mNodes[2] = v3;

  bool repeated[3] = {false, false, false};
  for (auto edg : mpTemplate->get_Edges()) {
    repeated[0] = repeated[0] or edg->IsEqual(v1, v2);
    repeated[1] = repeated[1] or edg->IsEqual(v2, v3);
    repeated[2] = repeated[2] or edg->IsEqual(v1, v3);
  }
  if (!repeated[0])
    new Edge(v1, v2, this, temp);
  if (!repeated[1])
    new Edge(v2, v3, this, temp);
  if (!repeated[2])
    new Edge(v1, v3, this, temp);

  texture.first = static_cast<KeyFrame *>(nullptr);
  R = 0.5;
  G = 0.5;
  B = 0.5;
}

Facet::~Facet() {
  Nodes.clear();
  mNodes[0] = static_cast<Node *>(nullptr);
  mNodes[1] = static_cast<Node *>(nullptr);
  mNodes[2] = static_cast<Node *>(nullptr);
  Edges.clear();
  mPoints.clear();
  Keyframes.clear();

  mpTemplate = static_cast<Template *>(nullptr);
}

std::set<Node *> Facet::getNodes() {
  //  unique_lock<mutex> lock(this->mMuteFacet);
  return Nodes;
}
std::array<Node *, 3> Facet::getNodesArray() { return mNodes; }

bool Facet::NoObservations() { return (mPoints.size() == 0); }
void Facet::addMapPoint(MapPoint *mp) { mPoints.insert(mp); }
void Facet::EraseMapPoint(MapPoint *mp) { mPoints.erase(mp); }

std::set<Edge *> Facet::getEdges() { return Edges; }

void Facet::SetBadFlag() {
  mpTemplate->EraseFacet(this);

  std::set<Node *> MapNodes = mpTemplate->get_Nodes();
  for (std::set<Node *>::iterator it = MapNodes.begin(); it != MapNodes.end();
       it++) {
    (*it)->deassigneFacet(this);
    if ((*it)->No_facets())
      (*it)->SetBadFlag();
  }
}
std::set<MapPoint *> Facet::getMapPoints() { return mPoints; }

double Facet::getArea() {
  std::vector<Eigen::Vector3d> nodes;
  for (std::set<Node *>::iterator it = Nodes.begin(); it != Nodes.end(); it++) {
    Eigen::Vector3d nod;
    nod << (*it)->x, (*it)->y, (*it)->z;
    nodes.push_back(nod);
  }

  double area = ((nodes[0] - nodes[1]).cross((nodes[2] - nodes[1]))).norm();
  return area;
}

void Facet::setKeyframes(std::set<KeyFrame *> keyframe) {
  std::unique_lock<std::mutex> mM(mMutexKeyframe);
  Keyframes = keyframe;
}

void Facet::getTextureCoordinates() {
  // Loop over the Keyframes to see a proper one

  texture.first = nullptr;
  //        std::cout << Keyframes.size() << std::endl;
  for (std::set<KeyFrame *>::iterator it = Keyframes.begin();
       it != Keyframes.end(); it++) {

    vector<cv::Point3f> Nodes3d;
    vector<cv::Point2f> Nodes2d;

    for (std::set<Node *>::iterator itn = Nodes.begin(); itn != Nodes.end();
         itn++) {
      double x, y, z;
      (*itn)->getXYZ(x, y, z);
      Nodes3d.push_back(cv::Point3f(float(x), float(y), float(z)));
    }
    cv::Mat mTcw = (*it)->GetPose();

    cv::Mat mK = (*it)->mK.clone();
    cv::Mat RotVect;
    cv::Mat rotMat = mTcw(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat transMat = mTcw(cv::Range(0, 3), cv::Range(3, 4));
    cv::Rodrigues(rotMat, RotVect);
    cv::projectPoints(Nodes3d, RotVect, transMat, mK, cv::noArray(), Nodes2d);

    bool IsIn(true);
    uint i(0);
    std::map<Node *, cv::Point> points;
    for (std::set<Node *>::iterator itn = Nodes.begin(); itn != Nodes.end();
         itn++) {
      points[(*itn)] = Nodes2d[i];
      (*itn)->proju = Nodes2d[i].x;
      (*itn)->projv = Nodes2d[i].y;
      // std::cout << Nodes2d[i].x << " " << Nodes2d[i].y << std::endl;
      if (!((Nodes2d[i].x < 640) && (Nodes2d[i].y < 480) &&
            (Nodes2d[i].x > 0) && (Nodes2d[i].y > 0))) {
        IsIn *= false;
      }
      i++;
    }

    if (IsIn) {
      texture.first = (*it);
      TexturesDataset[*it].insert(this);
      texture.second = points;
      break;
    }
  }
}

void Facet::getTextureCoordinates(KeyFrame *KF) {
  // Loop over the Keyframes to see a proper one

  texture.first = nullptr;
  //        std::cout << Keyframes.size() << std::endl;

  vector<cv::Point3f> Nodes3d;
  vector<cv::Point2f> Nodes2d;

  for (std::set<Node *>::iterator itn = Nodes.begin(); itn != Nodes.end();
       itn++) {
    double x, y, z;
    (*itn)->getXYZ(x, y, z);
    Nodes3d.push_back(cv::Point3f(float(x), float(y), float(z)));
  }
  cv::Mat mTcw = KF->GetPose();

  cv::Mat mK = KF->mK.clone();
  cv::Mat RotVect;
  cv::Mat rotMat = mTcw(cv::Range(0, 3), cv::Range(0, 3));
  cv::Mat transMat = mTcw(cv::Range(0, 3), cv::Range(3, 4));
  cv::Rodrigues(rotMat, RotVect);
  cv::projectPoints(Nodes3d, RotVect, transMat, mK, cv::noArray(), Nodes2d);

  bool IsIn(true);
  uint i(0);
  std::map<Node *, cv::Point> points;
  for (std::set<Node *>::iterator itn = Nodes.begin(); itn != Nodes.end();
       itn++) {
    points[*itn] = Nodes2d[i];
    (*itn)->proju = Nodes2d[i].x;
    (*itn)->projv = Nodes2d[i].y;
    if (!((Nodes2d[i].x < 640) && (Nodes2d[i].y < 480) && (Nodes2d[i].x > 0) &&
          (Nodes2d[i].y > 0))) {
      IsIn *= false;
    }
    i++;
  }

  if (IsIn) {
    texture.first = KF;
    TexturesDataset[KF].insert(this);
    texture.second = points;
  }
}
} // namespace ORB_SLAM
