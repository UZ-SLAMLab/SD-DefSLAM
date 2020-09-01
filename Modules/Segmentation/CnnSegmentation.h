//
// Created by user on 31/8/20.
//

#ifndef DEFORMABLESLAM_CNNSEGMENTATION_H
#define DEFORMABLESLAM_CNNSEGMENTATION_H

#include <torch/torch.h>
#include <torch/script.h>

#include<opencv2/core/core.hpp>

namespace defSLAM {

    class CnnSegmentation {
    public:
        CnnSegmentation() : mean_(0.485 * 255, 0.456 * 255, 0.406 * 255),
                            std_(0.229 * 255, 0.224 * 255, 0.225 * 255) {};

        void loadModel(std::string path);

        cv::Mat forward(const cv::Mat &im);

    private:
        cv::Mat checkImageSize(const cv::Mat &im);

        cv::Mat preprocessMat(const cv::Mat &im);

        std::vector<torch::jit::IValue> convertFromMatToTorch(const cv::Mat &im);

        cv::Mat convertFromTorchToMat(at::Tensor &tensor);

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
