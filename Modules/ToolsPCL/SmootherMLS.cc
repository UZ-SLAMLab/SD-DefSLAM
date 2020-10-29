/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

/**
* This file is part of DefSLAM.
*
* Copyright (C) 2017-2020 Jose Lamarca Peiro <jlamarca at unizar dot es>, J.M.M. Montiel (University
*of Zaragoza) && Shaifali Parashar, Adrien Bartoli (Université Clermont Auvergne)
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

#include <SmootherMLS.h>

#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/common/distances.h>

#include <pcl/filters/radius_outlier_removal.h>

// Constructor.
SmootherMLS::SmootherMLS(int polynomialOrder_, double searchRadius_)
    : polynomialOrder_(polynomialOrder_), searchRadius_(searchRadius_){

                                          };

using namespace pcl;

std::vector<int> SmootherMLS::outlierRemovalRadius(const SmootherMLS::pointcloud &pcVector)
{
    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());

    for (uint i(0); i < pcVector.size(); i++)
    {
        PointXYZ pclpoint;
        pclpoint.x = pcVector[i][0];
        pclpoint.y = pcVector[i][1];
        pclpoint.z = pcVector[i][2];
        cloud->push_back(pclpoint);
    }

    // Set parameters
    pcl::RadiusOutlierRemoval<PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(searchRadius_);
    outrem.setMinNeighborsInRadius(20);
    std::vector<int> indices;
    outrem.filter(indices);
    return indices;
}