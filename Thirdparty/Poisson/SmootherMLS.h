#pragma once

#include <vector>
#include <iostream>



class SmootherMLS{
    /************************************* 
     * Class to smooth using Moving Least Squares
     * 
     * args: 
     *    polynomialOrder_: polynomial fitting order
     *    searchRadius_: radius of point to fit the polinomial.
     * methods:
     *    smoothPointCloud:  function to smooth pointclouds
     *    input : std::vector<std::vector<float>> point cloud to smooth
     *    output : std::vector<std::vector<float>> smoothed point cloud 
     * 
     * Author: Jose Lamarca
     * Inspired in https://pcl-tutorials.readthedocs.io/en/latest/resampling.html?highlight=resampling#smoothing-and-normal-estimation-based-on-polynomial-reconstruction
     * 
     **********************************/
public:
    typedef std::vector<std::vector<float>> pointcloud; 

    SmootherMLS(int polynomialOrder_ = 2,double searchRadius_ = 0.03);

    std::vector<int> smoothPointCloud(const pointcloud& );

    std::vector<int> outlierRemovalRadius(const SmootherMLS::pointcloud& );
private:
    int polynomialOrder_;
    double searchRadius_;
};