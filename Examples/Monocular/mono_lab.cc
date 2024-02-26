/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ThreeDimensionalFrame.h"
// #include "Metrics.h"

using namespace std;

void LoadImages(const string &str11Sequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps,
                vector<bool> &vIsHorizontal,
                bool shouldUsePreprocessedFrames,
                vector<string> &vstrToolsImageFilenames,
                vector<string> &vstrToolsOrgansImageFilenames,
                int modeDynamic = 3,
                int slamMode = 3,
                int skeletMode = 0);

int main(int argc, char **argv)
{
    int modeDynamic = 3; //   0 static, 1 tools with DL, 2 tools + organs with own, 3 tools + organs with SAME
    int samMode = 3;
    int skeletMode = 2;     // 0 normal //1 all paths  (tip le plus proche du centre) //2 prend le point le plkus proche sur le contour le plus long
    int STARTIMAGE = 13200; // 3000;
    bool shouldUsePreprocessedFrames = 1;
    bool shouldErasePointsFromMap = 0; // points are are only prevented from being added to the map, but not removed from the map....

    if (argc != 5)
    {
        cerr << endl
             << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // ORB_SLAM2::Metrics metrics = ORB_SLAM2::Metrics(string(argv[3]), modeDynamic);

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<string> vstrToolsImageFilenames;
    vector<string> vstrToolsOrgansImageFilenames;
    vector<double> vTimestamps;
    vector<bool> vIsHorizontal;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps, vIsHorizontal, shouldUsePreprocessedFrames, vstrToolsImageFilenames, vstrToolsOrgansImageFilenames, modeDynamic, samMode, skeletMode);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, argv[3], true, modeDynamic, skeletMode);
    // ORB_SLAM2::System SLAMO(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, argv[3], true, true);
    //  std::string resultPath = argv[3] + string("results/outputGridFused.mp4");
    //  int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    //  double fps = 30.0;
    //  cv::VideoWriter fusedGridWriter = cv::VideoWriter(resultPath, fourcc, fps, cv::Size(480, 270));
    //  cv::namedWindow("Fused Grid", cv::WINDOW_AUTOSIZE);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    std::cout << std::endl
              << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << nImages << std::endl
              << std::endl;

    // Main loop
    cv::Mat im;
    cv::Mat preprocessToolsIm;
    cv::Mat preprocessToolsOrgansIm;

    for (int ni = STARTIMAGE; ni < nImages; ni++)
    {
        std::cout << "______________________________" << std::endl;
        std::cout << "Frame " << ni << std::endl;
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        if (modeDynamic == 1 || modeDynamic == 3)
        {
            std::cout << "reading " << vstrToolsImageFilenames[ni] << std::endl;
            preprocessToolsIm = cv::imread(vstrToolsImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
            if (shouldUsePreprocessedFrames)
                cv::imshow("Preprocessed Tools", preprocessToolsIm);
        }
        if (modeDynamic == 3)
        {
            std::cout << "reading " << vstrToolsOrgansImageFilenames[ni] << std::endl;
            preprocessToolsOrgansIm = cv::imread(vstrToolsOrgansImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
            if (shouldUsePreprocessedFrames)
                cv::imshow("Preprocessed Tools Organs", preprocessToolsOrgansIm);
        }
        double tframe = vTimestamps[ni];

        if (im.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif

        // Pass the image to the SLAM system
        cv::Mat Tcw = SLAM.TrackMonocular(im, tframe, preprocessToolsIm, preprocessToolsOrgansIm);
        // metrics.updatePosition(Tcw, vIsHorizontal[ni], ni);
        //  SLAMO.TrackMonocular(im, tframe);
        //   cv::Mat fusedGrid = SLAMO.grid->projectGrid(SLAM.mCurrentGrid, true, true);
        //   fusedGridWriter.write(fusedGrid);
        //   if (fusedGrid.size().width > 0 && fusedGrid.size().height > 0)
        //       cv::imshow("Fused Grid", fusedGrid);
        //   double mT = 1e3 / fps;
        //   char key = cv::waitKey(mT);

        // // if key is q
        // if (key == 'q')
        // {
        //     fusedGridWriter.release();
        //     usleep(6000);
        //     exit(0);
        // }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();
    // SLAMO.Shutdown();

    // Tracking time statistics
    std::sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    std::cout << "-------" << std::endl
              << std::endl;
    std::cout << "median tracking time: " << vTimesTrack[nImages / 2] << std::endl;
    std::cout << "mean tracking time: " << totaltime / nImages << std::endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    // SLAMO.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectoryO.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, vector<bool> &vIsHorizontal, bool shouldUsePreprocessedFrames, vector<string> &vstrToolsImageFilenames, vector<string> &vstrToolsOrgansImageFilenames, int modeDynamic, int samMode, int skeletMode)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    fTimes.close();

    ifstream fIsHorizontal;
    string strPathIsHorizontalFile = strPathToSequence + "/isHorizontal.txt";

    // test if file exists
    if (!ifstream(strPathIsHorizontalFile))
    {
        std::cout << "isHorizontal.txt not found" << std::endl;
    }
    else
    {
        fIsHorizontal.open(strPathIsHorizontalFile.c_str());
        while (!fIsHorizontal.eof())
        {
            string s;
            getline(fIsHorizontal, s);
            if (!s.empty())
            {
                stringstream ss;
                ss << s;
                bool isHorizontal;
                ss >> isHorizontal;
                vIsHorizontal.push_back(isHorizontal);
            }
        }
        fIsHorizontal.close();
    }

    string strPrefixLeft = strPathToSequence + "/frames/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
    vstrToolsImageFilenames.resize(nTimes);
    vstrToolsOrgansImageFilenames.resize(nTimes);

    if (shouldUsePreprocessedFrames)
    {
        //// TOOLS
        if (modeDynamic == 1 || modeDynamic == 3)
        {
            string strPrefixTools = strPathToSequence + "/preSegmentedFramesTools/";

            for (int i = 0; i < nTimes; i++)
            {
                stringstream ss;
                ss << setfill('0') << setw(6) << i;
                vstrToolsImageFilenames[i] = strPrefixTools + ss.str() + ".png";
            }
        }
        if (modeDynamic == 3)
        {
            string strPrefixToolsOrgans = strPathToSequence + "/preSegmentedFramesToolsOrgans/sam" + to_string(samMode) + "/skelet" + to_string(skeletMode) + "/";

            for (int i = 0; i < nTimes; i++)
            {
                stringstream ss;
                ss << setfill('0') << setw(6) << i;
                vstrToolsOrgansImageFilenames[i] = strPrefixToolsOrgans + ss.str() + ".png";
            }
        }
    }
}
