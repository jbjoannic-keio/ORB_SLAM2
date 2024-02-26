#ifndef POSITION_WRITER_H
#define POSITION_WRITER_H

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace ORB_SLAM2
{

    class PositionWriter
    {
    public:
        PositionWriter(const std::string &strPath, const int mode = 0);
        void writePosition(cv::Mat &Tcw, const int state, const double &timestamp);
        void resetSlam(const double timestamp);
        void closeFile();

    private:
        int iMode;
        FILE *pFile;
    };
}; // namespace ORB_SLAM2

#endif // THREE_DIMENSIONAL_FRAME_H