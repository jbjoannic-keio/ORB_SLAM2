#ifndef DYNAMIC_ERASER_H
#define DYNAMIC_ERASER_H

#include <iostream>
#include <string>
#include <random>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm/fundamental.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>

#include "Frame.h"
#include "MapPoint.h"
namespace ORB_SLAM2
{
    class DynamicEraser
    {
    public:
        DynamicEraser(cv::Mat mK);
        std::vector<std::pair<cv::Point2f, cv::Point2f>> searchMatchesKeyFrame(Frame &CurrentFrame, Frame &LastFrame);

        static std::pair<cv::Mat, cv::Mat> convertToMatrix(std::vector<std::pair<cv::Point2f, cv::Point2f>> matches);
        static std::vector<int> randomWhichPoints(int matchesMatrixSize);
        static cv::Mat computeFundamental(std::pair<cv::Mat, cv::Mat> matchesMatrix, std::vector<int> whichPoints);
        static std::vector<float> computeVector(cv::Mat F, cv::Mat K);
        std::vector<std::pair<cv::Point2f, cv::Point2f>> Ransac(std::vector<std::pair<cv::Point2f, cv::Point2f>> matches);
        std::pair<std::vector<std::pair<cv::Point2f, cv::Point2f>>, std::vector<std::pair<cv::Point2f, cv::Point2f>>> RealRansac(std::vector<std::pair<cv::Point2f, cv::Point2f>> matches);
        static float computeDistance(std::vector<float> vec1, std::vector<float> vec2);
        int main();
        cv::Mat K;
    };

} // namespace ORB_SLAM2

#endif // DYNAMIC_ERASER_H