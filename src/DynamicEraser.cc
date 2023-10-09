#include "DynamicEraser.h"

namespace ORB_SLAM2
{
    DynamicEraser::DynamicEraser(cv::Mat mK)
    {
        // std::cout << "DynamicEraser::DynamicEraser" << std::endl;
        DynamicEraser::K = mK;
    };

    std::vector<std::pair<cv::Point2f, cv::Point2f>> DynamicEraser::searchMatchesKeyFrame(Frame &CurrentFrame, Frame &LastFrame)
    {
        // std::cout << "DynamicEraser::Erase" << std::endl;
        // std::cout << "CurrentFrame.timestamp = " << CurrentFrame.mTimeStamp << std::endl;
        // std::cout << "LastFrame.timestamp = " << LastFrame.mTimeStamp << std::endl;

        std::vector<std::pair<cv::Point2f, cv::Point2f>> matches;
        for (int i = 0; i < CurrentFrame.N; i++)
        // for (int i = 0; i < 10; i++)
        {
            // continue; /////////////////////////////////?              ERASE DAT

            if (CurrentFrame.mvpMapPoints[i] && !CurrentFrame.mvbDynamicOutlier[i])
            {
                MapPoint *pMP = CurrentFrame.mvpMapPoints[i];

                if (pMP)
                {
                    cv::Point2f ptCurrent, ptLast;
                    // std::cout << "i = " << i << std::endl;
                    /////std::cout << "pMP->Observations() = " << pMP->Observations() << std::endl;
                    if (pMP->GetObservations().size() < 1)
                        continue;

                    for (auto const &pair : pMP->GetObservations())
                    {
                        // std::cout << "pair.first->timestamp = " << pair.first->mTimeStamp << std::endl;
                        // std::cout << "X " << pair.first->mvKeysUn[pair.second].pt.x << "Y " << pair.first->mvKeysUn[pair.second].pt.y << std::endl;
                        // std::cout << "pair.second keyidpoint = " << pair.second << std::endl;
                    }

                    std::vector<std::pair<KeyFrame *, size_t>> vec;

                    for (auto const &pair : pMP->GetObservations())
                    {
                        vec.emplace_back(pair.first, pair.second);
                    }

                    // reverse sort by timestamp of the pair
                    std::sort(vec.begin(), vec.end(),
                              [](const std::pair<KeyFrame *, size_t> &a, const std::pair<KeyFrame *, size_t> &b)
                              {
                                  return a.first->mTimeStamp > b.first->mTimeStamp;
                              });
                    for (auto const &pair : vec)
                    {
                        // std::cout << "SORTED pair.first->timestamp = " << pair.first << std::endl;
                        // std::cout << "VEC X " << pair.first->mvKeysUn[pair.second].pt.x << "Y " << pair.first->mvKeysUn[pair.second].pt.y << std::endl;
                        // std::cout << "SORTED pair.second keyidpoint = " << pair.second << std::endl;
                    }

                    if (i > CurrentFrame.mvKeysUn.size())
                        continue;
                    // std::cout << "pt " << CurrentFrame.mvKeysUn[i].pt << " size " << vec.size() << endl;
                    ptCurrent.x = CurrentFrame.mvKeysUn[i].pt.x;
                    ptCurrent.y = CurrentFrame.mvKeysUn[i].pt.y;

                    ptLast.x = vec[0].first->mvKeysUn[vec[0].second].pt.x;
                    ptLast.y = vec[0].first->mvKeysUn[vec[0].second].pt.y;

                    matches.emplace_back(std::make_pair(ptCurrent, ptLast));
                }
            }
        }
        return matches;
    };

    std::pair<cv::Mat, cv::Mat> DynamicEraser::convertToMatrix(std::vector<std::pair<cv::Point2f, cv::Point2f>> matches)
    {
        cv::Mat pointsLast(matches.size(), 2, CV_32F), pointsCurrent(matches.size(), 2, CV_32F);

        for (int i = 0; i < matches.size(); i++)
        {
            pointsCurrent.at<float>(i, 0) = matches[i].first.x;
            pointsCurrent.at<float>(i, 1) = matches[i].first.y;
            pointsLast.at<float>(i, 0) = matches[i].second.x;
            pointsLast.at<float>(i, 1) = matches[i].second.y;
        }
        return std::make_pair(pointsCurrent, pointsLast);
    }

    std::vector<int> DynamicEraser::randomWhichPoints(int matchesMatrixSize)
    {
        std::vector<int> whichPoints(matchesMatrixSize, 0);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, matchesMatrixSize - 1);
        for (int i = 0; i < 8; i++)
        {
            int random = dis(gen);
            whichPoints[random] = 1;
        }
        return whichPoints;
    }

    cv::Mat DynamicEraser::computeFundamental(std::pair<cv::Mat, cv::Mat> matchesMatrix, std::vector<int> whichPoints)
    {
        // std::cout << "DynamicEraser::computeFundamental" << std::endl;

        int count = 0;
        for (int i = 0; i < whichPoints.size(); i++)
        {
            if (whichPoints[i] == 1)
            {
                count++;
            }
        }
        cv::Mat subsetCurrent(count, 2, CV_32F), subsetLast(count, 2, CV_32F);
        cv::Mat F;

        int j = 0;
        for (int i = 0; i < whichPoints.size(); i++)
        {
            if (whichPoints[i] == 1)
            {
                subsetCurrent.at<float>(j, 0) = matchesMatrix.first.at<float>(i, 0);
                subsetCurrent.at<float>(j, 1) = matchesMatrix.first.at<float>(i, 1);
                subsetLast.at<float>(j, 0) = matchesMatrix.second.at<float>(i, 0);
                subsetLast.at<float>(j, 1) = matchesMatrix.second.at<float>(i, 1);
                j++;
            }
        }
        F = cv::findFundamentalMat(subsetLast, subsetCurrent, cv::FM_8POINT);
        // cv::sfm::normalizedEightPointSolver(subsetLast.t(), subsetCurrent.t(), F);
        // std::cout << subsetLast.t() << std::endl;
        // std::cout << subsetCurrent.t() << std::endl;
        // std::cout << "fin fundamental F = " << std::endl
        //          << F << std::endl;
        return F;
    }

    std::vector<float> DynamicEraser::computeVector(cv::Mat F, cv::Mat mK)
    {
        // std::cout << "debut vectorisation" << std::endl;
        cv::Mat E;
        cv::Mat R = cv::Mat::zeros(3, 3, CV_32F); // FAUT METTRE DE|S VECTORS ?????
        cv::Mat T = cv::Mat::zeros(1, 3, CV_32F);
        cv::Mat K(mK);
        std::vector<cv::Mat> Rotations;
        for (int i = 0; i < 4; i++)
        {
            Rotations.push_back(cv::Mat::zeros(3, 3, CV_64F));
        }                                  // = {cv::Mat::zeros(3, 3, CV_32F), cv::Mat::zeros(3, 3, CV_32F), cv::Mat::zeros(3, 3, CV_32F), cv::Mat::zeros(3, 3, CV_32F)};
        std::vector<cv::Mat> Translations; // = {cv::Mat::zeros(1, 3, CV_32F), cv::Mat::zeros(1, 3, CV_32F), cv::Mat::zeros(1, 3, CV_32F), cv::Mat::zeros(1, 3, CV_32F)};
        for (int i = 0; i < 4; i++)
        {
            Rotations.push_back(cv::Mat::zeros(3, 1, CV_64F));
        }
        // std::cout << "mid" << std::endl;
        cv::sfm::essentialFromFundamental(F, K, K, E);
        // std::cout << "mid2" << std::endl;

        cv::sfm::motionFromEssential(E, Rotations, Translations);
        // std::cout << "mid3" << std::endl;

        // std::cout << "F =" << std::endl
        //            << F << std::endl;
        //  std::cout << "K =" << std::endl
        //            << K << std::endl;
        //  std::cout << "E =" << std::endl
        //            << E << std::endl;
        for (int i = 0; i < 4; i++)
        {
            std::cout << Rotations[i] << std::endl;
            std::cout << Translations[i] << std::endl;
        }
        cv::Mat vecR(1, 3, CV_32F);
        // std::cout << "mid#" << std::endl;

        std::vector<float> vec(6);
        cv::Rodrigues(Rotations[0], vecR);
        // std::cout << "vecR = " << std::endl;
        for (int i = 0; i < 3; i++)
        {
            std::cout << vecR.at<float>(i) << "  ";
        }
        std::cout << std::endl;
        vec[0] = vecR.at<float>(0);
        vec[1] = vecR.at<float>(1);
        vec[2] = vecR.at<float>(2);
        vec[3] = Translations[0].at<float>(0);
        vec[4] = Translations[0].at<float>(1);
        vec[5] = Translations[0].at<float>(2);
        for (int i = 0; i < 6; i++)
        {
            std::cout << vec[i] << "  ";
        }
        // std::cout << std::endl;
        // std::cout << "fin vectorisation" << std::endl;
        return vec;
    }

    float DynamicEraser::computeDistance(std::vector<float> vec1, std::vector<float> vec2)
    {
        float distance = 0;
        for (int i = 0; i < vec1.size(); i++)
        {
            distance += pow(vec1[i] - vec2[i], 2);
        }
        return sqrt(distance);
    }

    std::vector<std::pair<cv::Point2f, cv::Point2f>> DynamicEraser::Ransac(std::vector<std::pair<cv::Point2f, cv::Point2f>> matches)
    {
        // std::cout << "RANSAC" << std::endl;
        std::pair<cv::Mat, cv::Mat> matchesMatrix = DynamicEraser::convertToMatrix(matches);
        // std::cout << "matrixed" << std::endl;
        //  std::vector<int> whichPoints = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}; // normalement taille de  laliste des matches mais on peut sarreter avant
        //  cv::Mat F = DynamicEraser::computeFundamental(matchesMatrix, whichPoints);
        //  std::vector<float> vec = DynamicEraser::computeVector(F);
        int iterMax = 1;            // nombre de fois qu'on fait le ransac
        float thresholdOutlier = 1; // seuil de distance outlier
        int iter = 0;
        int bestInliersNumber = 0;
        std::vector<int> bestWhichPoints;
        while (iter < iterMax)
        {
            std::vector<int> whichPoints = DynamicEraser::randomWhichPoints(matches.size());
            // std::cout << "randomize" << std::endl;
            cv::Mat F = DynamicEraser::computeFundamental(matchesMatrix, whichPoints);
            // std::cout << "Fundamental" << std::endl;
            std::vector<float> vec = DynamicEraser::computeVector(F, K);
            // std::cout << "vector" << std::endl;
            // std::cout << "size " << whichPoints.size() << std::endl;
            for (int i = 0; i < whichPoints.size(); i++)
            {
                if (whichPoints[i] == 0)
                {
                    whichPoints[i] = 1;
                    // std::cout << "deb i=0 boucle n " << i << std::endl;
                    cv::Mat FProvisoire = DynamicEraser::computeFundamental(matchesMatrix, whichPoints);
                    // std::cout << "Fundamental 2 boucle n " << i << std::endl;
                    std::vector<float> vecProvisoire = DynamicEraser::computeVector(FProvisoire, K);
                    // std::cout << "vector 2 boucle n " << i << std::endl;
                    float distance = DynamicEraser::computeDistance(vec, vecProvisoire);
                    // std::cout << "distance 2 boucle n " << i << std::endl;
                    if (distance > thresholdOutlier)
                    {
                        whichPoints[i] = 0;
                    }
                }
            }
            int count = 0;
            for (int i = 0; i < whichPoints.size(); i++)
            {
                if (whichPoints[i] == 1)
                {
                    count++;
                }
            }
            if (count > bestInliersNumber)
            {
                bestInliersNumber = count;
                bestWhichPoints = std::vector<int>(whichPoints);
                // std::cout << "bestInliersNumber : " << bestInliersNumber << std::endl;
            }
            iter++;
        }
        std::vector<std::pair<cv::Point2f, cv::Point2f>> inliers;
        for (int i = 0; i < bestWhichPoints.size(); i++)
        {
            if (bestWhichPoints[i] == 1)
            {
                inliers.push_back(matches[i]);
            }
        }

        return inliers;
    }

    std::pair<std::vector<std::pair<cv::Point2f, cv::Point2f>>, std::vector<std::pair<cv::Point2f, cv::Point2f>>> DynamicEraser::RealRansac(std::vector<std::pair<cv::Point2f, cv::Point2f>> matches)
    {
        // std::cout << "RANSAC" << std::endl;
        std::pair<cv::Mat, cv::Mat> matchesMatrix = DynamicEraser::convertToMatrix(matches);
        // std::cout << "matrixed" << std::endl;
        cv::Mat mask;
        cv::Mat F = cv::findFundamentalMat(matchesMatrix.first, matchesMatrix.second, cv::FM_RANSAC, 3, 0.99, mask);
        // std::cout << "Fundamental" << std::endl;
        // std::cout << F << std::endl;
        // std::cout << "Fundamental" << std::endl;
        // std::cout << mask << std::endl;
        // std::cout << "mask" << std::endl;
        std::vector<std::pair<cv::Point2f, cv::Point2f>> outliers;
        std::vector<std::pair<cv::Point2f, cv::Point2f>> inliers;
        for (int i = 0; i < mask.rows; i++)
        {
            if (mask.at<uchar>(i, 0) == 0)
            {
                outliers.push_back(matches[i]);
            }
            else
            {
                inliers.push_back(matches[i]);
            }
        }
        return std::make_pair(inliers, outliers);
    }

    // https://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf

    /*double rotationDistance(cv::Mat r1)
    {
        // OMEGA 2
        // TO VECTOR
        cv::Vec3d vecAngle;
        cv::Rodrigues(r1, vecAngle);
    };

    double translationDistance(cv::Mat t1)
    {
        // OMEGA 1
        // distance
        cv::Vec3d vecTrans = t1.clone();
    };

    double totalDistance(){

    };

    void erase(std::vector<std::pair<cv::Point2f, cv::Point2f>> matches)
    {
    }*/

    int DynamicEraser::main()
    {
        std::vector<cv::Mat> rotationMatrices;
        for (int i = 0; i < 10; i++)
        {
            double angle = static_cast<double>(std::rand()) / RAND_MAX * 360.0;
            double angle2 = static_cast<double>(std::rand()) / RAND_MAX * 360.0;
            double angle3 = static_cast<double>(std::rand()) / RAND_MAX * 360.0;
            cv::Mat rotationMatrix;
            cv::Rodrigues(cv::Vec3f(angle, angle3, angle2), rotationMatrix);
            rotationMatrices.push_back(rotationMatrix);
        }
        // Convert the rotation matrices to rotation vectors
        // std::vector<cv::Vec3d> rotationVectors;
        cv::Mat rotationVectors(rotationMatrices.size(), 3, CV_32F);
        std::cout << "rotationVectors rows" << rotationVectors.rows << std::endl;
        int i = 0;
        for (const auto &rotationMatrix : rotationMatrices)
        {
            cv::Mat rotationVector(3, 1, CV_32F);
            cv::Rodrigues(rotationMatrix, rotationVector);

            for (int j = 0; j < rotationVector.rows; j++)
            {
                rotationVectors.at<float>(i, j) = rotationVector.at<float>(j, 0);
            }
            ///////////PB ICI
            i++;
        }
        // rotationVectors = rotationVectors.t();
        std::cout << "rotationVectors" << rotationVectors << std::endl;
        cv::Mat meanVector, covarianceMatrix;
        cv::Mat rotationVectorsMat(rotationVectors);
        cv::calcCovarMatrix(rotationVectorsMat, covarianceMatrix, meanVector, cv::COVAR_NORMAL | cv::COVAR_ROWS);
        std::cout << "rotationVectorsMat" << rotationVectorsMat << std::endl;
        std::cout << "meanVector" << meanVector << std::endl;
        std::cout << "covarianceMatrix" << covarianceMatrix << std::endl;

        // Invert the covariance matrix
        cv::Mat inverseCovarianceMatrix;
        std::cout << cv::determinant(covarianceMatrix) << std::endl;
        cv::invert(covarianceMatrix, inverseCovarianceMatrix);
        std::cout << "InvertcovarianceMatrix" << inverseCovarianceMatrix << std::endl;

        // Calculate the Mahalanobis distance for each rotation vector
        std::vector<float> mahalanobisDistances;
        for (int i = 0; i < rotationVectors.rows; i++)
        {
            // cv::Mat rotationVector = rotationVectors.row(i);
            // cv::Mat rotationVectorMat(rotationVector);
            // cv::Mat difference = rotationVector - meanVector;
            // cv::Mat mahalanobisDistance = difference * inverseCovarianceMatrix * difference.t();
            // mahalanobisDistances.push_back(mahalanobisDistance.at<double>(0, 0));
            // std::cout << "mahalanobolis" << mahalanobisDistance.at<double>(0, 0) << std::endl;
        }
        /*
        // Find the outliers based on the Mahalanobis distance
        double threshold = 3.0; // Set the threshold to 3 standard deviations from the mean
        for (int i = 0; i < rotationMatrices.size(); i++)
        {
            if (mahalanobisDistances[i] > threshold * threshold)
            {
                std::cout << "Outlier detected: " << rotationMatrices[i] << std::endl;
            }
        }
        */

        return 0;
    }
}