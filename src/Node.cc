#include "Node.h"

#include <mutex>

namespace defSLAM {
Node::Node(double x, double y, double z, uint indx, Template *temp)
    : x(x), y(y), z(z), xO(x), yO(y), zO(z), indx(indx), mTemplate(temp),
      role(NONOBS), Boundary(false), registred(false), PosVecDrawer(-1),
      proju(0), projv(0) {
  viewed = false;
  local = false;
}

Node::~Node() {
  mTemplate = nullptr;
  weights.clear();
  NodesjJ_1J.clear();
}

uint Node::get_Index() { return indx; }

void Node::set_Index(uint i) { indx = i; }

void Node::reset() {
  std::unique_lock<mutex> as(mMutexPos);
  x = xO;
  y = yO;
  z = zO;
}

void Node::AssignEdge(Edge *edge) { mEdge.insert(edge); }

void Node::AssignFacet(Facet *facet) { mFacet.insert(facet); }

void Node::EraseFacet(Facet *facet) { mFacet.erase(facet); }

std::set<Edge *> Node::Get_Edges() { return mEdge; }

double Node::Distanceto(Node *node) {
  double d = pow(this->x - node->x, 2) + pow(this->y - node->y, 2) +
             pow(this->z - node->z, 2);
  return sqrt(d);
}

void Node::deassigneFacet(Facet *face) { mFacet.erase(face); }

bool Node::No_facets() { return (mFacet.size() == 0); }

void Node::update() {
  std::unique_lock<std::mutex> a(mMutexCond);
  if (VIEWED == role) {
    local = false;
    viewed = true;
  } else if (LOCAL == role) {
    viewed = false;
    local = true;
  } else {
    viewed = false;
    local = false;
  }
}

void Node::SetBadFlag() {
  mTemplate->EraseNode(this);
  for (std::set<Edge *>::iterator it = mEdge.begin(); it != mEdge.end(); it++) {
    (*it)->SetBadFlag();
  }
}

std::set<Node *> Node::GetNeighbours() {
  std::set<Node *> Neighbours;
  for (std::set<Edge *>::iterator Edges = mEdge.begin(); Edges != mEdge.end();
       Edges++) {
    std::set<Node *> EdgeNodes = (*Edges)->get_nodes();
    for (std::set<Node *>::iterator Nodes = EdgeNodes.begin();
         Nodes != EdgeNodes.end(); Nodes++) {
      if (*Nodes != this)
        Neighbours.insert(*Nodes);
    }
  }
  return Neighbours;
}

void Node::deassignEdge(Edge *edge) { mEdge.erase(edge); }

void Node::setViewed() {
  std::unique_lock<std::mutex> a(mMutexCond);
  role = VIEWED;
}

void Node::setLocal() {
  std::unique_lock<std::mutex> a(mMutexCond);
  if (role != VIEWED)
    role = LOCAL;
}

void Node::resetRole() {
  std::unique_lock<std::mutex> a(mMutexCond);
  role = NONOBS;
}

bool Node::isViewed() {
  std::unique_lock<std::mutex> a(mMutexCond);
  return viewed;
}
bool Node::isLocal() {
  std::unique_lock<std::mutex> a(mMutexCond);
  return local;
}

void Node::setXYZ(double xx, double yy, double zz) {
  std::unique_lock<std::mutex> a(mMutexPos);
  this->x = xx;
  this->y = yy;
  this->z = zz;
}

void Node::getXYZ(double &xx, double &yy, double &zz) {
  std::unique_lock<std::mutex> a(mMutexPos);
  xx = this->x;
  yy = this->y;
  zz = this->z;
}

void Node::getInitialPose(double &xx, double &yy, double &zz) {
  xx = xO;
  yy = yO;
  zz = zO;
}

void Node::setBoundary() { Boundary = true; }

bool Node::isBoundary() { return Boundary; }

std::set<Facet *> Node::GetFacets() { return mFacet; }
} // namespace ORB_SLAM
