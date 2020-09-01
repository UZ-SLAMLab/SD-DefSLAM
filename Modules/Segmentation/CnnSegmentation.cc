//
// Created by user on 31/8/20.
//

#include "CnnSegmentation.h"

#include <torch/script.h>
#include <opencv2/opencv.hpp>

namespace defSLAM {

    void CnnSegmentation::loadModel(std::string path) {
        this->model_ = torch::jit::load(path);
        std::cout << "Loaded model at " << path << std::endl;
    }

    cv::Mat CnnSegmentation::forward(const cv::Mat &im) {
        //Resize image if necessary
        cv::Mat resizedIm = this->checkImageSize(im);

        //Preprocess resized image
        cv::Mat processedIm = this->preprocessMat(resizedIm);

        //Convert Mat to torch
        auto cnnInput = this->convertFromMatToTorch(processedIm);

        //Call neural network
        auto cnnOutput = model_.forward(cnnInput).toTensor();

        //Convert back to Mat
        cv::Mat rawMask = this->convertFromTorchToMat(cnnOutput);

        //Postprocess image
        cv::Mat mask = this->postprocessMat(rawMask);

        //Resize to match the original size
        cv::resize(mask, mask, im.size());

        return mask.clone();
    }

    cv::Mat CnnSegmentation::checkImageSize(const cv::Mat &im) {
        cv::Size newSize = im.size(), originalSize = im.size();

        float fractional;
        if (modf(float(originalSize.width) / (float) SIZE_DIV_, &fractional) > 0.f)
            newSize.width = originalSize.width / SIZE_DIV_ * SIZE_DIV_;

        if (modf(float(originalSize.height) / (float) SIZE_DIV_, &fractional) > 0.f)
            newSize.height = originalSize.height / SIZE_DIV_ * SIZE_DIV_;

        cv::Mat resizedIm;
        cv::resize(im, resizedIm, newSize);

        return resizedIm.clone();
    }

    cv::Mat CnnSegmentation::preprocessMat(const cv::Mat &im) {
        cv::Mat out;

        im.copyTo(out);
        out.convertTo(out, CV_32FC3);

        //Subtract mean
        cv::subtract(out, mean_, out);

        //Divide by std
        cv::divide(out, std_, out);

        return out.clone();
    }

    std::vector<torch::jit::IValue> CnnSegmentation::convertFromMatToTorch(const cv::Mat &im) {
        auto input_tensor = torch::from_blob(im.data, {im.rows, im.cols, im.channels()});
        input_tensor = input_tensor.permute({2, 0, 1});

        auto unsqueezed = input_tensor.unsqueeze(0);

        std::vector <torch::jit::IValue> torchVector;
        torchVector.push_back(unsqueezed);

        return torchVector;
    }

    cv::Mat CnnSegmentation::convertFromTorchToMat(at::Tensor &tensor) {
        cv::Mat mat(tensor.size(2), tensor.size(3), CV_32F, (void *) tensor.data<float>());
        return mat.clone();
    }

    cv::Mat CnnSegmentation::postprocessMat(const cv::Mat &im) {
        cv::Mat out;

        cv::threshold(im, out, 0.f, 255, cv::THRESH_BINARY_INV);
        out.convertTo(out, CV_8U);

        return out.clone();
    }
}