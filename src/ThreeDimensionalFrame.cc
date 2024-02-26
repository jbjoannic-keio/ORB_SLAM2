#include "ThreeDimensionalFrame.h"

namespace ORB_SLAM2
{
    ThreeDimensionalFrame::ThreeDimensionalFrame(const std::string &strSettingPath, const std::string &strPath, const int mode)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        std::string windowName = "RAW";
        if (mode != 0)
        {
            windowName += " Outliers Removed";
        }
        iMode = mode;
        // cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    };

    void ThreeDimensionalFrame::createGrid(float x1, float x2, float y1, float y2, float z1, float z2)
    {

        std::vector<int> line_direction;
        int count = 0;

        if (z1 != z2)
            count += (x2 - x1 + 1) * (y2 - y1 + 1);
        if (x1 != x2)
            count += (y2 - y1 + 1) * (z2 - z1 + 1);
        if (y1 != y2)
            count += (x2 - x1 + 1) * (z2 - z1 + 1);
        Eigen::MatrixX4f original(count * 2, 4);
        count = 0;

        if (z1 != z2)
        {
            for (float x = x1; x <= x2; ++x)
            {
                for (float y = y1; y <= y2; ++y)
                {

                    original.row(count) << x, y, z1, 1;
                    original.row(count + 1) << x, y, z2, 1;
                    line_direction.push_back(2);
                    line_direction.push_back(2);
                    count += 2;
                }
            }
        }
        if (x1 != x2)
        {
            for (float y = y1; y <= y2; ++y)
            {
                for (float z = z1; z <= z2; ++z)
                {

                    original.row(count) << x1, y, z, 1;
                    original.row(count + 1) << x2, y, z, 1;
                    line_direction.push_back(0);
                    line_direction.push_back(0);
                    count += 2;
                }
            }
        }
        if (y1 != y2)
        {
            for (float x = x1; x <= x2; ++x)
            {
                for (float z = z1; z <= z2; ++z)
                {

                    original.row(count) << x, y1, z, 1;
                    original.row(count + 1) << x, y2, z, 1;
                    line_direction.push_back(1);
                    line_direction.push_back(1);
                    count += 2;
                }
            }
        }
        threeDimGrid = std::make_pair(original, line_direction);
    };

    void ThreeDimensionalFrame::computeGridRotation(cv::Mat &Tcw)
    {
        Eigen::MatrixX4f originalCopy = threeDimGrid.first;
        rotationGridPoints = Eigen::MatrixX4f(threeDimGrid.first.rows(), 4);

        cv::Mat Rwc = cv::Mat::zeros(4, 4, CV_32F);

        Rwc.rowRange(0, 3).colRange(0, 3) = Tcw.rowRange(0, 3).colRange(0, 3).t();
        Rwc.at<float>(3, 3) = 1;

        cv::Mat t = Tcw.rowRange(0, 3).col(3);
        cv::Mat twc = -Tcw.rowRange(0, 3).colRange(0, 3).t() * t;
        // std::cout << twc << std::endl;
        Rwc.at<float>(0, 3) = twc.at<float>(0);
        Rwc.at<float>(1, 3) = twc.at<float>(1);
        Rwc.at<float>(2, 3) = twc.at<float>(2);
        // Rwc.rowRange(0, 3).col(3) = twc;

        Tcw.at<float>(0, 3) = Tcw.at<float>(0, 3) * 10;
        Tcw.at<float>(1, 3) = Tcw.at<float>(1, 3) * 10;
        Tcw.at<float>(2, 3) = Tcw.at<float>(2, 3) * 10;

        Eigen::Map<Eigen::Matrix4f> RcwEigen(Rwc.ptr<float>(), 4, 4);
        Eigen::Map<Eigen::Matrix4f> TcwEigen(Tcw.ptr<float>(), 4, 4);
        // originalCopy.array().col(0) += std::fmod(Tcw.at<float>(0, 3), 10);
        // originalCopy.array().col(2) += std::fmod(Tcw.at<float>(2, 3), 10);
        rotationGridPoints = originalCopy * TcwEigen; //.transpose();

        // print first line of rotation grid point
        // std::cout << Rwc << std::endl;
        // std::cout << originalCopy.row(0) << std::endl;

        // std::cout << rotationGridPoints.row(0) << std::endl;
    };

    void ThreeDimensionalFrame::correctGridRotation()
    {
        Eigen::MatrixX4f rotationGridPointsCopy = rotationGridPoints;
        std::vector<int> correctedLine_direction;

        for (int i = 0; i < rotationGridPoints.rows(); i += 2)
        {
            if (rotationGridPoints(i, 2) < 0.1 && rotationGridPoints(i + 1, 2) > 0.1)
            {
                // remove point behind camera
                float ratio = (0.1 - rotationGridPoints(i + 1, 2)) / (rotationGridPoints(i, 2) - rotationGridPoints(i + 1, 2));
                rotationGridPointsCopy.row(i) = ratio * (rotationGridPointsCopy.row(i) - rotationGridPointsCopy.row(i + 1)) + rotationGridPointsCopy.row(i + 1);
            }
            if (rotationGridPoints(i + 1, 2) < 0.1 && rotationGridPoints(i, 2) > 0.1)
            {
                // remove point behind camera
                float ratio = (0.1 - rotationGridPoints(i, 2)) / (rotationGridPoints(i + 1, 2) - rotationGridPoints(i, 2));
                rotationGridPointsCopy.row(i + 1) = ratio * (rotationGridPointsCopy.row(i + 1) - rotationGridPointsCopy.row(i)) + rotationGridPointsCopy.row(i);
            }

            if (rotationGridPoints(i + 1, 2) < 0.1 && rotationGridPoints(i, 2) < 0.1)
            {
                // remove both points
                rotationGridPointsCopy.row(i) << 0, 0, 0, 0;
                rotationGridPointsCopy.row(i + 1) << 0, 0, 0, 0;
            }
        }
        Eigen::VectorXi mask = rotationGridPointsCopy.rowwise().all().cast<int>();
        Eigen::MatrixX4f corrected(mask.count(), 4);
        int j = 0;
        for (int i = 0; i < mask.size(); i++)
            if (mask(i) == 1)
            {
                corrected.row(j++) = rotationGridPointsCopy.row(i);
                correctedLine_direction.push_back(threeDimGrid.second[i]);
            }
        correctedRotationGridPoints = std::make_pair(corrected, correctedLine_direction);
    };

    cv::Mat ThreeDimensionalFrame::projectGrid(const cv::Mat &img, const bool removeDynamicOutliers, const bool isFusedGrid)
    {
        cv::Mat copy;
        if (img.channels() == 1)
            cvtColor(img, copy, CV_GRAY2RGB);
        else
            copy = img.clone();

        Eigen::Map<Eigen::Matrix3f> mKEigen(mK.ptr<float>(), 3, 3);
        cv::Scalar color;
        switch (iMode)
        {
        case 0:
            color = cv::Scalar(100, 150, 0);
            break;
        case 1:
            color = cv::Scalar(100, 0, 150);
            break;
        case 2:
            color = cv::Scalar(150, 0, 100);
            break;
        case 3:
            color = cv::Scalar(150, 100, 0);
            break;
        }
        std::vector<cv::Scalar> colors = {cv::Scalar(100, 100, 100), cv::Scalar(255, 255, 255), color};
        std::vector<int> thickness = {1, 1, 2};

        projectedGridPoints = Eigen::MatrixX3f(correctedRotationGridPoints.first.rows(), 3);
        projectedGridPoints = correctedRotationGridPoints.first.leftCols(3) * mKEigen;
        projectedGridPoints.col(0) = projectedGridPoints.col(0).array() / projectedGridPoints.col(2).array();
        projectedGridPoints.col(1) = projectedGridPoints.col(1).array() / projectedGridPoints.col(2).array();

        for (int i = 0; i < projectedGridPoints.rows(); i += 2)
        {
            cv::line(
                copy,
                cv::Point(projectedGridPoints(i, 0), projectedGridPoints(i, 1)),
                cv::Point(projectedGridPoints(i + 1, 0), projectedGridPoints(i + 1, 1)),
                colors[correctedRotationGridPoints.second[i]],
                thickness[correctedRotationGridPoints.second[i]]);
        }

        // std::string windowName = "RAW";
        // if (removeDynamicOutliers)
        // {
        //     windowName += " Outliers Removed";
        // }
        // if (!isFusedGrid)
        // {
        //     if (copy.size().width > 0 && copy.size().height > 0)
        //         cv::imshow(windowName, copy);
        // }
        return copy;
    }

}
