#include "Metrics.h"

namespace ORB_SLAM2
{
    Metrics::Metrics(const std::string &strPath, const int mode)
    {
        iMode = mode;
        std::string additionalString = "";
        switch (mode)
        {
        case 0:
            additionalString = "_base";
            break;
        case 1:
            additionalString = "_toolsRemoved";
            break;
        case 2:
            additionalString = "_toolsAndOrgansRemoved";
            break;
        case 3:
            additionalString = "_toolsAndOrgansRemovedSAM";
            break;
        }
        std::string fileName = strPath + "/results/metrics" + additionalString + ".csv";
        pFile = fopen(fileName.c_str(), "w");

        if (pFile == NULL)
        {
            std::cout << "Error opening file" << std::endl;
            exit(1);
        }
        else
        {
            std::cout << "File opened successfully" << std::endl;
        }

        fprintf(pFile, "frameIdStart,angularError\n");
    };

    void Metrics::updatePosition(cv::Mat Tcw, bool isHorizontal, int frameId)
    {
        std::cout << "Metrics::updatePosition" << std::endl;
        std::cout << "Tcw: " << Tcw << std::endl;
        cv::Mat visualizeIsHorizontal = cv::Mat::zeros(100, 100, CV_8UC3);
        if (askedReset)
        {
            isHorizontal = false;
        }
        if (isHorizontal)
        {
            if (!wasHorizontal)
            {
                wasHorizontal = true;
                frameIdStart = frameId;
            }
            // green image
            visualizeIsHorizontal = cv::Mat::zeros(100, 100, CV_8UC3);
            visualizeIsHorizontal.setTo(cv::Scalar(0, 255, 0));
            // test avec metrique pas folle seulement sur la rota
            if (Tcw.size().width == 0)
            {
                std::cout << "Tcw.size().width == 0" << std::endl;
                Eigen::Matrix3d eigenR;

                eigenR << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;
                double trace = eigenR.trace();
                double theta = std::acos((trace - 1.) / 2.);
                std::cout << "error: " << theta << std::endl;
                fprintf(pFile, "%d,%f\n", frameId, -2.);
                // errorSum += error;
            }
            else
            {
                Eigen::Matrix<double, 3, 3> eigenR;
                cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
                eigenR << Rcw.at<float>(0, 0), Rcw.at<float>(0, 1), Rcw.at<float>(0, 2),
                    Rcw.at<float>(1, 0), Rcw.at<float>(1, 1), Rcw.at<float>(1, 2),
                    Rcw.at<float>(2, 0), Rcw.at<float>(2, 1), Rcw.at<float>(2, 2);

                double trace = eigenR.trace();
                double theta = std::acos((trace - 1.) / 2.);
                std::cout << "error: " << theta << std::endl;
                fprintf(pFile, "%d,%f\n", frameId, theta);
                errorSum += theta;
                nbError++;
            }
        }
        else
        {
            // red image
            visualizeIsHorizontal = cv::Mat::zeros(100, 100, CV_8UC3);
            visualizeIsHorizontal.setTo(cv::Scalar(0, 0, 255));
            if (wasHorizontal)
            {
                vErrorsPerSegment.push_back(std::make_pair(std::vector<int>{frameIdStart, frameId - 1}, errorSum / nbError));
                errorSum = 0;
                nbError = 0;
                wasHorizontal = false;
                frameIdStart = 0;
            }
        }
        if (askedReset)
        {
            vErrorsPerSegment.push_back(std::make_pair(std::vector<int>{frameId, frameId}, -1));
            askedReset = false;
            fprintf(pFile, "%d,%f\n", frameId, -1.);
        }
        // limite sur les nb de frame pour pas exploser la ram
        if (isHorizontal && nbError > 300)
        {
            vErrorsPerSegment.push_back(std::make_pair(std::vector<int>{frameIdStart, frameId}, errorSum / nbError));
            errorSum = 0;
            nbError = 0;
            wasHorizontal = false;
            frameIdStart = 0;
        }
        std::cout << "vErrorsPerSegment: ";
        for (int i = 0; i < vErrorsPerSegment.size(); i++)
        {
            std::cout << vErrorsPerSegment[i].first[0] << " " << vErrorsPerSegment[i].first[1] << " " << vErrorsPerSegment[i].second << " | ";
        }
        std::cout << std::endl;
        cv::imshow("isHorizontal", visualizeIsHorizontal);
    }

    void Metrics::reset()
    {
        errorSum = 0;
        nbError = 0;
        wasHorizontal = false;
        frameIdStart = 0;
        askedReset = true;
    }

    void Metrics::closeFile()
    {
        fclose(pFile);
    }
}