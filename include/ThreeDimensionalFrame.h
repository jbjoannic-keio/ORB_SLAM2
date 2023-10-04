#ifndef THREE_DIMENSIONAL_FRAME_H
#define THREE_DIMENSIONAL_FRAME_H

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
namespace ORB_SLAM2
{
    // This class is used to create a 3D grid and project it into the image plane
    class ThreeDimensionalFrame
    {
    public:
        // Constructor, use the setting paths to get the camera parameters
        ThreeDimensionalFrame(const std::string &strSettingPath, const std::string &strPath, const bool removeDynamicOutliers = false);

        // Create a 3D grid with the given parameters (boundaries of the grid around the camera)
        void createGrid(float x1, float x2, float y1, float y2, float z1, float z2);

        // compute the points of the grid after applying the rotation
        void computeGridRotation(cv::Mat &Tcw);

        // correct the points of the grid after applying the rotation by erasing the points behind the camera, and recreating lines extremities
        void correctGridRotation();

        // project the grid into the image plane
        cv::Mat projectGrid(const cv::Mat &img, const bool removeDynamicOutliers = false, const bool isFusedGrid = false);

    private:
        // The grid is stored as a pair of a matrix containing the points of the grid and a vector containing the direction of each line
        // The matrix is composed of 4 columns, the first 3 are the coordinates of the point, the last one is a 1.
        // Each point is the extremity of the line, and function with his pair.
        // The below matrix represents two lines, one from (1,0,0) to (1,0,1) and the other from (1,1,1) to (1,2,0)
        // [0] 1,0,0,1
        // [1] 1,0,1,1
        // [2] 1,1,1,1
        // [3] 1,2,0,1
        std::pair<Eigen::MatrixX4f, std::vector<int>> threeDimGrid;

        // The grid after applying the rotation (just the matrix without directions, use the same nomenclature as the original grid)
        Eigen::MatrixX4f rotationGridPoints;

        // The grid after applying the rotation and correcting the points behind the camera (just the matrix without directions, use the same nomenclature as the original grid)
        std::pair<Eigen::MatrixX4f, std::vector<int>> correctedRotationGridPoints;

        // the intrinseque parameters of the camera
        cv::Mat mK;

        // matrix containing the img coordinates of each points
        Eigen::MatrixX3f projectedGridPoints;
    };

} // namespace ORB_SLAM2

#endif // THREE_DIMENSIONAL_FRAME_H