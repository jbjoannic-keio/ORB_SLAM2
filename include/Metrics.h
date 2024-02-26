#ifndef METRICS_H
#define METRICS_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>>
#include <Eigen/Dense>
#include <iostream>

namespace ORB_SLAM2
{
    class Metrics
    {
    private:
        double errorSum = 0;
        int nbError = 0;
        bool wasHorizontal = false;
        int frameIdStart = 0;
        bool askedReset = false;
        FILE *pFile;
        int iMode;

    public:
        Metrics(const std::string &strPath, const int mode = 0);
        std::vector<std::pair<std::vector<int>, double>> vErrorsPerSegment;
        void updatePosition(cv::Mat Tcw, bool isHorizontal, int frameId);
        void reset();
        void closeFile();
    };
} // namespace ORB_SLAM2

#endif // METRICS_H