#pragma once

#include <vector>


class PCLNormalEstimator{
public: 	
	PCLNormalEstimator() = default;

	// Points introduced as [X0,Y0,Z0,X1,Y1,Z1,...,Xi,Yi,Zi] in the same vector
    PCLNormalEstimator(std::vector<float> & Points,double radious = 0.3);

	// Normals given as [X0,Y0,Z0,X1,Y1,Z1,...,Xi,Yi,Zi] in the same vector
	std::vector<float> && getNormals();
private:

    std::vector<float> normals;

};
