#ifndef ROBOT_SURGERY_SEGMENTATION_H
#define ROBOT_SURGERY_SEGMENTATION_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <torch/torch.h>
#include <torch/script.h>
#include <filesystem>
#include <vector>

namespace ORB_SLAM2
{
    class RobotSurgerySegmentation
    {
    public:
        RobotSurgerySegmentation(std::string modelPath, bool isBig);
        cv::Mat mask(cv::Mat imagesrc);
        cv::Mat fuse(cv::Mat image, cv::Mat mask);

    private:
        torch::jit::script::Module model;
        bool isBig;
    };
} // namespace ORB_SLAM2

#endif // ROBOT_SURGERY_SEGMENTATION_H