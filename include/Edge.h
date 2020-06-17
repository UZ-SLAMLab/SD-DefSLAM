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

#ifndef EDGE_H
#define EDGE_H

#include <Template.h>
#include <Node.h>
#include <Facet.h>
#include <set>
#include <mutex>

namespace defSLAM {

class Template;
class Node;
class Facet;

class Edge 
{
public:
  Edge(Template *);
  Edge(Node *, Node *, Facet *, Template *);
  ~Edge();
  bool operator==(const Edge &);
  bool IsEqual(const Node *, const Node *);
  double get_dist();
  std::pair<uint, uint> get_pair_nodes();
  std::set<Node *> get_nodes();
  void addFacet(Facet *facet);
  void EraseFacet(Facet *facet);
  std::set<Facet *> getfacets();
  void deassigneFacet(Facet *facet);
  bool No_facets();
  void SetBadFlag();
  Template *getTemplate() { return mpTemplate; }

private:
  Template *mpTemplate;
  std::pair<uint, uint> edge;
  std::set<Node *> mspNode;

  Node *mNodes[2];
  std::set<Facet *> facetsConected;
  double InitialDist;
};

} // namespace defSLAM

#endif // EDGE_H
