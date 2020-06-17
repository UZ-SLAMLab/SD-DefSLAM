#include <Edge.h>
#include <mutex>

namespace defSLAM {
Edge::Edge(Template *temp) : mpTemplate(temp) {
  std::vector<Node *> a;
  for (std::set<Node *>::iterator it = mspNode.begin(); it != mspNode.end();
       it++) {
    a.push_back(*it);
  }
  InitialDist = a[0]->Distanceto(a[1]);
}

Edge::Edge(Node *v1, Node *v2, Facet *facet, Template *temp) {
  mNodes[0] = v1;
  mNodes[1] = v2;
  mspNode.insert(v1);
  mspNode.insert(v2);

  bool RepeatedEdge(false);
  const std::set<Edge *> &MapEdges = temp->get_Edges();
  for (std::set<Edge *>::const_iterator it = MapEdges.begin();
       it != MapEdges.end(); it++) {
    if ((*it)->operator==(*this)) {
      RepeatedEdge = true;
      break;
    }
  }

  if (!RepeatedEdge) {
    InitialDist = v1->Distanceto(v2);
    mpTemplate = temp;
    temp->AddEdge(this);
    this->addFacet(facet);
    (v1)->AssignEdge(this);
    (v2)->AssignEdge(this);
  }
}

Edge::~Edge() {
  mspNode.clear();
  mpTemplate = nullptr;
  mNodes[0] = nullptr;
  mNodes[1] = nullptr;
  facetsConected.clear();
}

double Edge::get_dist() { return InitialDist; }

std::pair<uint, uint> Edge::get_pair_nodes() {
  std::vector<Node *> a;
  for (std::set<Node *>::iterator it = mspNode.begin(); it != mspNode.end();
       it++) {
    a.push_back(*it);
  }
  return {a[0]->get_Index(), a[1]->get_Index()};
}

void Edge::addFacet(Facet *facet) { facetsConected.insert(facet); }
void Edge::EraseFacet(Facet *facet) { facetsConected.erase(facet); }
std::set<Facet *> Edge::getfacets() { return facetsConected; }
std::set<Node *> Edge::get_nodes() { return mspNode; }

void Edge::deassigneFacet(Facet *face) { facetsConected.erase(face); }

bool Edge::No_facets() { return (facetsConected.size() == 0); }

void Edge::SetBadFlag() {
  mpTemplate->EraseEdge(this);
  for (std::set<Node *>::iterator it = mspNode.begin(); it != mspNode.end();
       it++) {
    (*it)->deassignEdge(this);
  }
}

bool Edge::operator==(const Edge &OtherEdge) {
  bool b1 = (this->mNodes[0] == OtherEdge.mNodes[0]) or
            (this->mNodes[0] == OtherEdge.mNodes[1]);
  bool b2 = (this->mNodes[1] == OtherEdge.mNodes[0]) or
            (this->mNodes[1] == OtherEdge.mNodes[1]);
  return b1 && b2;
}

bool Edge::IsEqual(const Node *v1, const Node *v2) {
  bool b1 = (this->mNodes[0] == v1) or (this->mNodes[0] == v2);
  bool b2 = (this->mNodes[1] == v1) or (this->mNodes[1] == v2);
  return b1 && b2;
}
} // namespace ORB_SLAM2
