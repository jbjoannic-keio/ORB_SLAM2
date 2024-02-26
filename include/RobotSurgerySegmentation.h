#ifndef ROBOT_SURGERY_SEGMENTATION_H
#define ROBOT_SURGERY_SEGMENTATION_H

#include </home/jbjoannic/Documents/Recherche/orbSlam/segment-anything-cpp-wrapper-1.5/sam.h>

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <torch/torch.h>
#include <torch/script.h>
#include <filesystem>
#include <vector>
#include "Frame.h"
#include "Graph.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

namespace ORB_SLAM2
{
    class RobotSurgerySegmentation
    {
    public:
        RobotSurgerySegmentation(std::string modelPath, bool isBig, std::string modelOrgansPath);
        cv::Mat maskOrgans(cv::Mat imagesrc, cv::Mat toolsMask);
        cv::Mat mask(cv::Mat imagesrc);
        cv::Mat fuse(cv::Mat frame, cv::Mat DLMask, cv::Mat DLMaskCenter, cv::Mat Reconstructed, cv::Mat NotReconstructed);
        std::vector<cv::Point> selectExtrimity(cv::Mat DLMask, cv::Mat image);
        std::vector<cv::Point> selectExtrimityBySkelet(cv::Mat DLMask);
        cv::Mat getSamMask(cv::Mat image, std::vector<cv::Point> baseSAM);

    private:
        torch::jit::script::Module modelOrgans;
        torch::jit::script::Module model;
        bool isBig;
        SEGMENT_ANYTHING::Sam *sam = nullptr;
        Graph graph;
        // cv::VideoWriter outputVideo;
        // int outputFrameNumber = 0;
    };
} // namespace ORB_SLAM2

#endif // ROBOT_SURGERY_SEGMENTATION_H