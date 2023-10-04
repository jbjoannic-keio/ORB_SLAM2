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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace ORB_SLAM2
{

    FrameDrawer::FrameDrawer(Map *pMap, const string strPath, const bool removeDynamicOutliers) : mpMap(pMap)
    {
        mState = Tracking::SYSTEM_NOT_READY;
        mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        double fps = 30.0;

        outGrid = cv::VideoWriter(strPath + "results/outputGrid" + (removeDynamicOutliers ? "_removed" : "") + ".mp4", fourcc, fps, cv::Size(480, 270));
        outAll = cv::VideoWriter(strPath + "results/outputAll" + (removeDynamicOutliers ? "_removed" : "") + ".mp4", fourcc, fps, cv::Size(480, 270));
        if (removeDynamicOutliers)
        {
            outInliers = cv::VideoWriter(strPath + "results/outputInliers.mp4", fourcc, fps, cv::Size(480, 270));
            outOutliers = cv::VideoWriter(strPath + "results/outputOutliers.mp4", fourcc, fps, cv::Size(480, 270));
            outInOutliers = cv::VideoWriter(strPath + "results/outputInOutliers.mp4", fourcc, fps, cv::Size(480, 270));
            outDL_small = cv::VideoWriter(strPath + "results/outputDL_small.mp4", fourcc, fps, cv::Size(480, 270));
            outDL_big = cv::VideoWriter(strPath + "results/outputDL_big.mp4", fourcc, fps, cv::Size(480, 270));
        }
    }

    std::vector<cv::Mat> FrameDrawer::DrawFrame()
    {
        std::vector<cv::Mat> vectIm;
        cv::Mat im, imInliers, imOutliers, imInOutliers;
        vector<cv::KeyPoint> vIniKeys;     // Initialization: KeyPoints in reference frame
        vector<int> vMatches;              // Initialization: correspondeces with reference keypoints
        vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
        vector<bool> vbVO, vbMap;          // Tracked MapPoints in current frame
        int state;                         // Tracking state

        // Copy variables within scoped mutex
        {
            unique_lock<mutex> lock(mMutex);
            state = mState;
            if (mState == Tracking::SYSTEM_NOT_READY)
                mState = Tracking::NO_IMAGES_YET;

            mIm.copyTo(im);

            mIm.copyTo(imInliers);
            mIm.copyTo(imOutliers);
            mIm.copyTo(imInOutliers);

            if (mState == Tracking::NOT_INITIALIZED)
            {
                vCurrentKeys = mvCurrentKeys;
                vIniKeys = mvIniKeys;
                vMatches = mvIniMatches;
            }
            else if (mState == Tracking::OK)
            {
                vCurrentKeys = mvCurrentKeys;
                vbVO = mvbVO;
                vbMap = mvbMap;
            }
            else if (mState == Tracking::LOST)
            {
                vCurrentKeys = mvCurrentKeys;
            }
        } // destroy scoped mutex -> release mutex

        if (im.channels() < 3) // this should be always true
            cvtColor(im, im, CV_GRAY2BGR);

        if (imInliers.channels() < 3) // this should be always true
            cvtColor(imInliers, imInliers, CV_GRAY2BGR);

        if (imOutliers.channels() < 3) // this should be always true
            cvtColor(imOutliers, imOutliers, CV_GRAY2BGR);

        if (imInOutliers.channels() < 3) // this should be always true
            cvtColor(imInOutliers, imInOutliers, CV_GRAY2BGR);

        // Draw
        if (state == Tracking::NOT_INITIALIZED) // INITIALIZING
        {
            for (unsigned int i = 0; i < vMatches.size(); i++)
            {
                if (vMatches[i] >= 0)
                {
                    cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt,
                             cv::Scalar(0, 255, 0));
                }
            }
        }
        else if (state == Tracking::OK) // TRACKING
        {
            mnTracked = 0;
            mnTrackedVO = 0;
            const float r = 5;
            const int n = vCurrentKeys.size();
            for (int i = 0; i < n; i++)
            {
                if (vbVO[i] || vbMap[i])
                {
                    cv::Point2f pt1, pt2;
                    pt1.x = vCurrentKeys[i].pt.x - r;
                    pt1.y = vCurrentKeys[i].pt.y - r;
                    pt2.x = vCurrentKeys[i].pt.x + r;
                    pt2.y = vCurrentKeys[i].pt.y + r;

                    // This is a match to a MapPoint in the map
                    if (vbMap[i])
                    {
                        cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
                        cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
                        // std::cout << "FrameDrawer::DrawFrame() - vbMap[i] = " << vCurrentKeys[i].pt << std::endl;
                        mnTracked++;
                    }
                    else // This is match to a "visual odometry" MapPoint created in the last frame
                    {
                        cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
                        cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
                        mnTrackedVO++;
                    }
                }
            }

            std::vector<double> deltaXRansacInliers;
            std::vector<double> deltaYRansacInliers;
            const int nCouplesRansacInliers = mvCurrentMatchesRansacInliers.size();
            for (int j = 0; j < nCouplesRansacInliers; j++)
            {
                cv::circle(imInliers, mvCurrentMatchesRansacInliers[j].first, 2, cv::Scalar(0, 0, 255), -1);
                cv::circle(imInliers, mvCurrentMatchesRansacInliers[j].second, 2, cv::Scalar(255, 0, 0), -1); // position before
                cv::arrowedLine(imInliers, mvCurrentMatchesRansacInliers[j].first, mvCurrentMatchesRansacInliers[j].second, cv::Scalar(0, 255, 0));
                deltaXRansacInliers.push_back(mvCurrentMatchesRansacInliers[j].first.x - mvCurrentMatchesRansacInliers[j].second.x);
                deltaYRansacInliers.push_back(mvCurrentMatchesRansacInliers[j].first.y - mvCurrentMatchesRansacInliers[j].second.y);
            }

            std::vector<double> deltaXRansacOutliers;
            std::vector<double> deltaYRansacOutliers;
            const int nCouplesRansacOutliers = mvCurrentMatchesRansacOutliers.size();
            for (int j = 0; j < nCouplesRansacOutliers; j++)
            {
                cv::circle(imOutliers, mvCurrentMatchesRansacOutliers[j].first, 2, cv::Scalar(0, 0, 255), -1);
                cv::circle(imOutliers, mvCurrentMatchesRansacOutliers[j].second, 2, cv::Scalar(255, 0, 0), -1); // position before
                cv::arrowedLine(imOutliers, mvCurrentMatchesRansacOutliers[j].first, mvCurrentMatchesRansacOutliers[j].second, cv::Scalar(0, 0, 255));
                deltaXRansacOutliers.push_back(mvCurrentMatchesRansacOutliers[j].first.x - mvCurrentMatchesRansacOutliers[j].second.x);
                deltaYRansacOutliers.push_back(mvCurrentMatchesRansacOutliers[j].first.y - mvCurrentMatchesRansacOutliers[j].second.y);
            }

            for (int j = 0; j < nCouplesRansacInliers; j++)
            {
                cv::arrowedLine(imInOutliers, mvCurrentMatchesRansacInliers[j].first, mvCurrentMatchesRansacInliers[j].second, cv::Scalar(0, 255, 0));
            }
            for (int j = 0; j < nCouplesRansacOutliers; j++)
            {
                cv::arrowedLine(imInOutliers, mvCurrentMatchesRansacOutliers[j].first, mvCurrentMatchesRansacOutliers[j].second, cv::Scalar(0, 0, 255));
            }
            // matplotlibcpp::scatter(deltaX, deltaY);
            // matplotlibcpp::show();
        }

        // DL Model
        // if imgDL size == 0, then do not draw
        cv::Mat imDL_small, imDL_big;

        if (imgDL_small.size() != cv::Size(0, 0))
        {
            imDL_small = imgDL_small.clone();
            imDL_big = imgDL_big.clone();
        }

        cv::Mat imWithInfo;
        DrawTextInfo(im, state, imWithInfo);
        vectIm.push_back(imWithInfo);
        vectIm.push_back(imInliers);
        vectIm.push_back(imOutliers);
        vectIm.push_back(imInOutliers);
        vectIm.push_back(imDL_small);
        vectIm.push_back(imDL_big);
        outAll.write(im);
        if (mvCurrentMatches.size() > 0)
        {
            outInliers.write(imInliers);
            outOutliers.write(imOutliers);
            outInOutliers.write(imInOutliers);
        }
        return vectIm;
    }

    void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
    {
        stringstream s;
        if (nState == Tracking::NO_IMAGES_YET)
            s << " WAITING FOR IMAGES";
        else if (nState == Tracking::NOT_INITIALIZED)
            s << " TRYING TO INITIALIZE ";
        else if (nState == Tracking::OK)
        {
            if (!mbOnlyTracking)
                s << "SLAM MODE |  ";
            else
                s << "LOCALIZATION | ";
            int nKFs = mpMap->KeyFramesInMap();
            int nMPs = mpMap->MapPointsInMap();
            s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
            if (mnTrackedVO > 0)
                s << ", + VO matches: " << mnTrackedVO;
        }
        else if (nState == Tracking::LOST)
        {
            s << " TRACK LOST. TRYING TO RELOCALIZE ";
        }
        else if (nState == Tracking::SYSTEM_NOT_READY)
        {
            s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
        }

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
        im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
        imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
        cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
    }

    void FrameDrawer::Update(Tracking *pTracker)
    {
        unique_lock<mutex> lock(mMutex);
        pTracker->mImGray.copyTo(mIm);
        mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
        N = mvCurrentKeys.size();
        mvbVO = vector<bool>(N, false);
        mvbMap = vector<bool>(N, false);
        mbOnlyTracking = pTracker->mbOnlyTracking;

        if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED)
        {
            mvIniKeys = pTracker->mInitialFrame.mvKeys;
            mvIniMatches = pTracker->mvIniMatches;
        }
        else if (pTracker->mLastProcessedState == Tracking::OK)
        {
            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                {
                    if (!pTracker->mCurrentFrame.mvbOutlier[i])
                    {
                        if (pMP->Observations() > 0)
                            mvbMap[i] = true;
                        else
                            mvbVO[i] = true;
                    }
                }
            }
        }
        mState = static_cast<int>(pTracker->mLastProcessedState);

        if (pTracker->couplesPoints.size() > 0)
        {
            mvCurrentMatches = pTracker->couplesPoints;
            mvCurrentMatchesRansacInliers = pTracker->couplesPointsRansacInliers;
            mvCurrentMatchesRansacOutliers = pTracker->couplesPointsRansacOutliers;
            removeDynamicOutliersMask = pTracker->removeDynamicOutliersMask;
        }

        // on va mettre ic le dl
    }

    void FrameDrawer::gridActualize(cv::Mat grid)
    {
        imgGrid = grid.clone();
        outGrid.write(imgGrid);
    }

    void FrameDrawer::drawDLModel(cv::Mat imDL_small, cv::Mat imDL_big)
    {
        imgDL_small = imDL_small.clone();
        imgDL_big = imDL_big.clone();
        if (outDL_small.isOpened())
        {
            outDL_small.write(imgDL_small);
            outDL_big.write(imgDL_big);
        }
    }

} // namespace ORB_SLAM
