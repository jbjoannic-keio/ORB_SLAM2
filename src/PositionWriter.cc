#include "PositionWriter.h"

namespace ORB_SLAM2
{
    PositionWriter::PositionWriter(const std::string &strPath, const int mode)
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
        }
        std::string fileName = strPath + "/results/positions" + additionalString + ".csv";
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

        fprintf(pFile, "timestamp, nState, Rxx, Rxy, Rxz, Ryx, Ryy, Ryz, Rzx, Rzy, Rzz, Tx, Ty, Tz\n");
    };

    void PositionWriter::writePosition(cv::Mat &Tcw, const int state, const double &timestamp)
    {
        float Rxx, Rxy, Rxz, Ryx, Ryy, Ryz, Rzx, Rzy, Rzz, Tx, Ty, Tz;

        if (Tcw.empty())
        {
            // std::cout << "Tcw is empty" << std::endl;
            Rxx = 0.;
            Rxy = 0.;
            Rxz = 0.;
            Ryx = 0.;
            Ryy = 0.;
            Ryz = 0.;
            Rzx = 0.;
            Rzy = 0.;
            Rzz = 0.;
            Tx = 0.;
            Ty = 0.;
            Tz = 0.;
        }
        else
        {
            // std::cout << "Tcw is not empty" << std::endl;
            Rxx = Tcw.at<float>(0, 0);
            Rxy = Tcw.at<float>(0, 1);
            Rxz = Tcw.at<float>(0, 2);
            Ryx = Tcw.at<float>(1, 0);
            Ryy = Tcw.at<float>(1, 1);
            Ryz = Tcw.at<float>(1, 2);
            Rzx = Tcw.at<float>(2, 0);
            Rzy = Tcw.at<float>(2, 1);
            Rzz = Tcw.at<float>(2, 2);
            Tx = Tcw.at<float>(0, 3);
            Ty = Tcw.at<float>(1, 3);
            Tz = Tcw.at<float>(2, 3);
        }

        fprintf(pFile, "%f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", timestamp, state, Rxx, Rxy, Rxz, Ryx, Ryy, Ryz, Rzx, Rzy, Rzz, Tx, Ty, Tz);
    };

    void PositionWriter::resetSlam(const double timestamp)
    {
        fprintf(pFile, "%f, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", timestamp, -2, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.);
    };

    void PositionWriter::closeFile()
    {
        fclose(pFile);
    };

}; // namespace ORB_SLAM2