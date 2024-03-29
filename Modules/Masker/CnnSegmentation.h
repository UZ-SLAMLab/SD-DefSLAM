/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#ifndef DEFORMABLESLAM_CNNSEGMENTATION_H
#define DEFORMABLESLAM_CNNSEGMENTATION_H

#include <torch/torch.h>
#include <torch/script.h>

#include<opencv2/core/core.hpp>

#include "Filter.h"

namespace defSLAM{

    class CnnSegmentation : public Filter{
        /*
         * This class allows to load a convolutional neural network saved as a tor::jit script for binary
         * image segmentation. It provides an easy way to infer from OpenCv types, taking care of image pre
         * and post processing and data structure translation.
         */
    public:
        CnnSegmentation() : mean_(0.485 * 255, 0.456 * 255, 0.406 * 255),
                            std_(0.229 * 255, 0.224 * 255, 0.225 * 255) {};

        /*
         * Loads a cnn model stored at path
         */
        void loadModel(std::string path);

        cv::Mat generateMask(const cv::Mat& im){
            return forward(im);
        }

        std::string getDescription();

    private:
        /*
         * Uses de loaded model to infer a mask from an image im
         */
        cv::Mat forward(const cv::Mat &im);

        /*
         * Checks that the size of the image im is divisible by 2⁵ and resizes it if necessary
         */
        cv::Mat checkImageSize(const cv::Mat &im);

        /*
         * Returns a per-channel normalitazed version of the image im
         */
        cv::Mat preprocessMat(const cv::Mat &im);

        /*
         * Converts an image im stored in OpenCV to a vector of torch::jit::IValue. This
         * is the data structure used by any torch cnn
         */
        std::vector<torch::jit::IValue> convertFromMatToTorch(const cv::Mat &im);

        /*
         * Converts a torch tensor to an OpenCV mat image
         */
        cv::Mat convertFromTorchToMat(at::Tensor &tensor);

        /*
         * Applies a pos-processing step of the data infered by the net. Internally applies a binary threshold
         * and converts the image to the correct data type
         */
        cv::Mat postprocessMat(const cv::Mat &im);

        //Image normalization
        cv::Scalar mean_;
        cv::Scalar std_;

        //Image resize
        const int SIZE_DIV_ = 1 << 5;

        torch::jit::script::Module model_;
    };
}

#endif //DEFORMABLESLAM_CNNSEGMENTATION_H
