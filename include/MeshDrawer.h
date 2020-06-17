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

#ifndef MESHDRAWER_H
#define MESHDRAWER_H

#include <opencv2/core.hpp>

#include <iostream>
#include <vector>
#include <set>
#include <mutex>

namespace defSLAM
{

class MeshDrawer{
public:
    MeshDrawer() = default;

    ~MeshDrawer() =default;

    void addTextureImage(cv::Mat im);

    void AddNode(std::vector<double> pos, std::vector<double> proj, int role);
    void AddEdge(std::vector<int> edge);
    void AddFacet(std::vector<int> fac);
    void DrawMesh(double alpha = 1, bool drawedges = true);

 private:
    std::vector<int> Facets; // One vector [V1_1 V1_2 V1_3 ..Vn_1 Vn_2 Vn_3 ... VN_1 VN_2 VN_3] Facets
    std::vector<int> Edges;  // One vector [V1_1 V1_2 ... Vn_1 Vn_2 ... VN_1 VN_2] Edges
    std::vector<double> Nodes; // One vector [V1 V2 ... Vn ... VN] Vertex position
    std::vector<int> NodesRole; // One vector [R1 R2 ... Rn ... RN] Nodes role

    cv::Mat Texture;
    std::vector<float> NodesProjection; // One vector with the 2d projection of the vertex [v1 v2 ... vn ... vN]

    unsigned int timestamp;
private:
    std::mutex mtemp;
};

}
#endif // MESHS_H
