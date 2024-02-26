#include "RobotSurgerySegmentation.h"

namespace ORB_SLAM2
{

    std::vector<float> computeMean(const cv::Mat &image)
    {
        cv::Mat HSV;
        cv::cvtColor(image, HSV, cv::COLOR_BGR2HSV);
        std::vector<float> mean;
        float meanH = 0;
        float meanS = 0;
        float meanV = 0;
        for (int i = 0; i < image.rows; i++)
        {
            for (int j = 0; j < image.cols; j++)
            {
                meanH += static_cast<int>(HSV.at<cv::Vec3b>(i, j)[0]);
                meanS += static_cast<int>(HSV.at<cv::Vec3b>(i, j)[1]);
                meanV += static_cast<int>(HSV.at<cv::Vec3b>(i, j)[2]);
            }
        }
        meanH /= (image.rows * image.cols);
        meanS /= (image.rows * image.cols);
        meanV /= (image.rows * image.cols);
        mean.push_back(meanH);
        mean.push_back(meanS);
        mean.push_back(meanV);
        return mean;
    }

    cv::Mat closestToCenterOfImageForEachArea(cv::Mat DLMask)
    {
        cv::Mat labeledImage;
        cv::Mat sumImage = cv::Mat::zeros(DLMask.size(), CV_8UC1);
        int numLabels = connectedComponents(DLMask, labeledImage, 8, CV_32S);
        for (int label = 1; label < numLabels; label++)
        {
            cv::Mat componentMask = (labeledImage == label);

            cv::Mat distanceMap(DLMask.size(), CV_32FC1, cv::Scalar(0));
            for (int i = 0; i < DLMask.rows; i++)
            {
                for (int j = 0; j < DLMask.cols; j++)
                {
                    if (componentMask.at<uchar>(i, j) > 0)
                    {
                        cv::Point point(j, i);
                        float distance = cv::norm(point - cv::Point(DLMask.cols / 2, DLMask.rows / 2));
                        distanceMap.at<float>(i, j) = distance;
                    }
                }
            }

            double maxVal;

            double minVal = std::numeric_limits<double>::max(); // Initialize with a large value

            for (int i = 0; i < DLMask.rows; i++)
            {
                for (int j = 0; j < DLMask.cols; j++)
                {
                    if (componentMask.at<uchar>(i, j) > 0)
                    {
                        double distance = distanceMap.at<float>(i, j);
                        if (distance < minVal)
                        {
                            minVal = distance;
                        }
                    }
                }
            }

            minMaxLoc(distanceMap, nullptr, &maxVal);
            double threshold = minVal + (maxVal - minVal) * 0.4;
            // mask is minValue < distanceMap < threshold

            cv::Mat mask = (distanceMap > minVal) & (distanceMap < threshold);

            cv::Mat resultImage = cv::Mat::zeros(DLMask.size(), CV_8UC1);
            resultImage.setTo(255, mask);
            max(resultImage, sumImage, sumImage);
        }
        return sumImage;
    }

    std::vector<cv::Mat> reconstruct(cv::Mat image, cv::Mat mask)
    {
        std::vector<cv::Mat> vec;
        cv::Mat M, Mwithout;
        cv::min(image, mask, M);
        cv::Scalar area = sum(M);
        cv::Mat strucElement = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        cv::Scalar s = 0;

        while (area[0] != s[0])
        {
            s = area;
            dilate(M, M, strucElement);
            min(image, M, M);
            area = sum(M);
        }
        Mwithout = image - M;
        vec.push_back(M);
        vec.push_back(Mwithout);
        cout << "fin reconstruct" << endl;
        return vec;
    }

    bool parseDeviceName(const std::string &name, SEGMENT_ANYTHING::Sam::Parameter::Provider &provider)
    {
        if (name == "cpu")
        {
            provider.deviceType = 0;
            return true;
        }
        if (name.substr(0, 5) == "cuda:")
        {
            provider.deviceType = 1;
            provider.gpuDeviceId = std::stoi(name.substr(5));
            return true;
        }
        return false;
    }

    RobotSurgerySegmentation::RobotSurgerySegmentation(std::string modelPath, bool big, std::string modelOrgansPath)
    {
        std::string modelName = modelPath.substr(modelPath.find_last_of("/") + 1, modelPath.find_last_of(".") - modelPath.find_last_of("/") - 1);

        model = torch::jit::load(modelPath);
        modelOrgans = torch::jit::load(modelOrgansPath);
        model.to(at::kCUDA);
        modelOrgans.to(at::kCUDA);
        isBig = big;

        int modelChoice = 3;

        std::string SamPreDevice = "cuda:0";
        std::string SamDevice = "cuda:0";

        std::vector<std::string> PreModels = {"/home/jbjoannic/Documents/Recherche/orbSlam/segment-anything-cpp-wrapper-1.5/models/preprocessed_vit_h.onnx",
                                              "/home/jbjoannic/Documents/Recherche/orbSlam/segment-anything-cpp-wrapper-1.5/models/preprocessed_vit_l.onnx",
                                              "/home/jbjoannic/Documents/Recherche/orbSlam/segment-anything-cpp-wrapper-1.5/models/preprocessed_vit_b.onnx",
                                              "/home/jbjoannic/Documents/Recherche/orbSlam/segment-anything-cpp-wrapper-1.5/models/preprocessed_mobile.onnx"};
        std::vector<std::string> Models = {"/home/jbjoannic/Documents/Recherche/orbSlam/segment-anything/ONXX/vit_h.onxx",
                                           "/home/jbjoannic/Documents/Recherche/orbSlam/segment-anything/ONXX/vit_l.onxx",
                                           "/home/jbjoannic/Documents/Recherche/orbSlam/segment-anything/ONXX/vit_b.onxx",
                                           "/home/jbjoannic/Documents/Recherche/orbSlam/MobileSAM/ONXX/mobile.onxx"};

        std::string SamPreModel = PreModels[modelChoice];
        std::string SamModel = Models[modelChoice];

        SEGMENT_ANYTHING::Sam::Parameter param(SamPreModel, SamModel, std::thread::hardware_concurrency());
        if (!parseDeviceName(SamPreDevice, param.providers[0]) ||
            !parseDeviceName(SamDevice, param.providers[1]))
        {
            std::cerr << "Unable to parse device name" << std::endl;
        }

        std::cout << "Loading SAM model... " << SamModel << std::endl;
        sam = new SEGMENT_ANYTHING::Sam(param); // FLAGS_pre_model, FLAGS_sam_model, std::thread::hardware_concurrency());
        std::cout << "constructreur" << sam->getInputSize() << std::endl;
        std::cout << "Model SAM loaded" << std::endl;

        // outputVideo = cv::VideoWriter("/home/jbjoannic/Documents/output.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(480, 270));
    }

    std::vector<cv::Point> RobotSurgerySegmentation::selectExtrimity(cv::Mat DLMask, cv::Mat image)
    {
        cv::Mat visualization = fuse(image, DLMask, cv::Mat::zeros(DLMask.size(), CV_8U), cv::Mat::zeros(DLMask.size(), CV_8U), cv::Mat::zeros(DLMask.size(), CV_8U));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(DLMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        int height = DLMask.rows;
        int width = DLMask.cols;
        cv::Point centerImage(width / 2, height / 2);

        std::vector<double> distancesShape;
        std::vector<double> distancesCenter;
        std::vector<cv::Point> baseSAM;
        for (const auto &contour : contours)
        {
            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);

            cv::Rect boundingRect = cv::boundingRect(contour);
            cv::Point center(boundingRect.x + boundingRect.width / 2, boundingRect.y + boundingRect.height / 2);
            // Moments M = moments(contour);
            // Point center(M.m10 / M.m00, M.m01 / M.m00);
            distancesShape.clear();
            distancesCenter.clear();
            double maxDistance = 0;
            double minDistance = std::numeric_limits<double>::max(); // Initialize with a large value
            for (const auto &point : hull)
            {
                cv::circle(visualization, point, 2, cv::Scalar(0, 0, 255), -1);
                double distance = cv::norm(center - point);
                distancesShape.push_back(distance);
                if (distance < minDistance)
                {
                    minDistance = distance;
                }
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                }
            }

            std::vector<int> max_distances_indices(distancesShape.size());
            std::iota(max_distances_indices.begin(), max_distances_indices.end(), 0);
            std::partial_sort(max_distances_indices.begin(), max_distances_indices.begin() + 1, max_distances_indices.end(),
                              [&distancesShape](int i1, int i2)
                              { return distancesShape[i1] > distancesShape[i2]; });
            double ratio = 0.4;
            std::vector<cv::Point> hull_extremities;
            for (int i = 0; i < distancesShape.size(); i++)
            {
                if (distancesShape[i] > maxDistance - (maxDistance - minDistance) * ratio)
                {
                    hull_extremities.push_back(hull[i]);
                    cv::line(visualization, center, hull[i], cv::Scalar(0, 0, 255), 1);
                    // min distance from borders
                    double distanceBorders = std::min(std::min(hull[i].x, width - hull[i].x), std::min(hull[i].y, height - hull[i].y));
                    // double distanceCenter = cv::norm(centerImage - hull[i]);
                    distancesCenter.push_back(distanceBorders);
                }
            }

            // Filter on distance to center
            std::vector<int> max_distances_indices_center(distancesCenter.size());

            for (int i = 0; i < distancesCenter.size(); i++)
            {
                max_distances_indices_center[i] = i;
            }
            std::sort(max_distances_indices_center.begin(), max_distances_indices_center.end(),
                      [&](int i1, int i2)
                      { return distancesCenter[i1] < distancesCenter[i2]; });

            std::vector<cv::Point> hull_extremities_center;
            // on prend le 1 premier
            // cout << "hull size: " << hull.size() << endl;
            // cout << "hull extr size: " << hull_extremities.size() << endl;
            // cout << "hull extr center size: " << max_distances_indices_center.size() << endl;
            for (int i = max_distances_indices_center.size() - 1; i >= 0; i--)
            {
                if (i < max_distances_indices_center.size() - 1)
                {
                    break;
                }
                // cout << "Point: " << hull_extremities[max_distances_indices_center[i]] << " " << i << " " << max_distances_indices_center[i] << " " << distancesCenter[max_distances_indices_center[i]] << endl;
                hull_extremities_center.push_back(hull_extremities[max_distances_indices_center[i]]);
            }
            // cout << "________________________" << endl;

            // vector<int> labels = DBSCAN(contour_extremities, 10, 2);

            // create convex hull object for each cluster
            // vector<vector<Point>> hull(contour_extremities.size());
            // for (int i = 0; i < contour_extremities.size(); i++)
            // {
            //     if (labels[i] != -1)
            //     {
            //         hull[labels[i]].push_back(contour_extremities[i]);
            //     }
            // }
            // drawContours(extremityMask, hull, -1, Scalar(255), 2);
            for (const auto &point : hull_extremities_center)
            {
                cv::line(visualization, point, center, cv::Scalar(255, 255, 255), 1);
            }
            std::vector<cv::Point> crossPoints;
            for (const auto &point : hull_extremities_center)
            {
                cv::Point2d pointD = point;
                cv::Point2d centerD = center;
                double distanceCenter = cv::norm(centerD - pointD);
                // point 10 px away from center
                cv::Point2d vectorGauche = (pointD - centerD) / distanceCenter; // vec unitaire

                cv::Point2d crossPoint = pointD + vectorGauche * 10;
                cv::Point2d vectorHaut = (pointD - centerD) / distanceCenter; // vec unitaire
                vectorHaut.x = -vectorGauche.y;
                vectorHaut.y = vectorGauche.x;
                cv::Point crossPointHaut = pointD + vectorHaut * 10;
                cv::Point crossPointBas = pointD - vectorHaut * 10;
                crossPoints.push_back(crossPoint);
                // crossPoints.push_back(crossPointHaut);
                // crossPoints.push_back(crossPointBas);
            }

            for (const auto &point : crossPoints)
            {
                cv::circle(visualization, point, 2, cv::Scalar(255, 255, 255), -1);
                baseSAM.push_back(point);
            }
        }
        imshow("extremityMask", visualization);
        return baseSAM;
    }

    std::vector<cv::Point> RobotSurgerySegmentation::selectExtrimityBySkelet(cv::Mat DLMask)
    {
        cv::Mat skelet = cv::Mat::zeros(DLMask.size(), CV_8UC1);
        cv::ximgproc::thinning(DLMask, skelet, cv::ximgproc::THINNING_GUOHALL);
        graph.createGraphFromSkeleton(skelet);
        graph.showGraphSimple(skelet);
        std::pair<cv::Mat, std::vector<cv::Point>> graphSkeletonResult = graph.getLongestSkeleton(skelet, 100);
        cv::Mat longestSkelet = graphSkeletonResult.first;
        std::vector<cv::Point> baseSam = graphSkeletonResult.second;

        return baseSam;
    }

    cv::Mat RobotSurgerySegmentation::getSamMask(cv::Mat image, std::vector<cv::Point> baseSAM)
    {
        int width = image.cols;
        cv::Size baseSize = image.size();
        int64 timeStart = cv::getTickCount();
        auto inputSize = sam->getInputSize();
        std::cout << "input size: " << inputSize << std::endl;
        std::cout << "image size: " << image.size() << std::endl;
        cv::resize(image, image, inputSize);
        cv::Mat sumMask = cv::Mat::zeros(image.size(), CV_8UC1);

        std::cout << "Loading image..." << std::endl;
        if (!sam->loadImage(image))
        {
            std::cout << "Image loading failed" << std::endl;
        }
        std::cout << "Image loaded" << std::endl;

        cv::Mat outImage = cv::Mat::zeros(image.size(), CV_8UC3);

        // multiple objects with one point, and not one object with multiple points
        for (cv::Point point : baseSAM)
        {
            cv::Point currentPoint;
            currentPoint = point * (inputSize.width / width);

            cv::Mat mask = sam->getMask(currentPoint, {});
            // no mask if too big
            // sumInliers
            int inlier = 0;
            int total = 589824;
            for (int i = 0; i < mask.rows; i++)
            {
                for (int j = 0; j < mask.cols; j++)
                {
                    if (mask.at<uchar>(i, j) > 0)
                    {
                        inlier++;
                    }
                }
            }
            if (inlier > total * 0.5)
            {
                continue;
            }
            sumMask = cv::max(sumMask, mask);
        }

        for (int i = 0; i < image.rows; i++)
        {
            for (int j = 0; j < image.cols; j++)
            {
                auto bFront = sumMask.at<uchar>(i, j) > 0;
                float factor = bFront ? 1.0 : 0.2;
                outImage.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(i, j) * factor;
            }
        }

        for (cv::Point point : baseSAM)
        {
            cv::Point currentPoint;
            currentPoint = point * (inputSize.width / width);
            cv::circle(outImage, currentPoint, 5, {0, 255, 0}, -1);
        }

        std::cout << "Mask computed" << std::endl;

        // apply mask to image

        cout << "time: " << (cv::getTickCount() - timeStart) / cv::getTickFrequency() << endl;
        timeStart = cv::getTickCount();
        cv::imshow("outImage", outImage);

        // if (outputFrameNumber < 300)
        // {
        //     cv::resize(outImage, outImage, cv::Size(480, 270));
        //     outputVideo.write(outImage);
        //     outputFrameNumber++;
        // }
        // else
        // {
        //     outputVideo.release();
        //     usleep(6000);
        //     exit(0);
        // }

        cv::resize(sumMask, sumMask, baseSize);
        return sumMask;
    }

    std::vector<cv::Mat> getRotatedElements(const cv::Mat &element, int nbRotations = 4)
    {
        std::vector<cv::Mat> rotatedElements;
        std::vector<double> angles;
        for (int i = 0; i < nbRotations; i++)
        {
            angles.push_back((i / float(nbRotations)) * 180);
        }
        for (int i = 0; i < angles.size(); i++)
        {
            cv::Mat rotatedElement;
            cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point((element.cols / 2), (element.rows / 2)), angles[i], 1);
            cv::warpAffine(element, rotatedElement, rotationMatrix, element.size());
            rotatedElements.push_back(rotatedElement);
        }
        return rotatedElements;
    }

    cv::Mat selectByMorphology(const cv::Mat &DLMask, cv::Size2i size_rectangle)
    {
        cv::Mat mask_opened, mask_opened_sum;
        mask_opened_sum = cv::Mat::zeros(DLMask.size(), CV_8UC1);
        int dim = max(size_rectangle.width, size_rectangle.height);    // pair
        int minDim = min(size_rectangle.width, size_rectangle.height); // impair
        cv::Mat rectangleElement = cv::Mat::zeros(cv::Size(dim + 1, dim + 1), CV_8UC1);
        // populate rectangle
        cv::rectangle(rectangleElement, cv::Point(0, (dim / 2) - (minDim - 1) / 2), cv::Point(dim, (dim / 2) + (minDim - 1) / 2), cv::Scalar(1), cv::FILLED);
        std::vector<cv::Mat> rotatedElements = getRotatedElements(rectangleElement, 8);
        for (auto element : rotatedElements)
        {
            cv::morphologyEx(DLMask, mask_opened, cv::MORPH_OPEN, element);
            cv::max(mask_opened, mask_opened_sum, mask_opened_sum);
        }

        // rotation struct element
        return mask_opened_sum;
    }

    cv::Mat selectByEccentricityAndConvexity(cv::Mat DLMask)
    {
        cv::Mat labeledImage;
        int numLabels = connectedComponents(DLMask, labeledImage, 8, CV_32S);

        cv::Mat filteredMask = cv::Mat::zeros(DLMask.size(), CV_8UC1);
        // cout << "_______________________________________________" << endl;
        // cout << "numLabels: " << numLabels << endl;

        for (int label = 1; label < numLabels; label++)
        {
            cv::Mat componentMask = (labeledImage == label);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(componentMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (contours[0].size() < 50)
            {
                continue;
            }

            std::vector<cv::Point> convexHull;
            cv::convexHull(contours[0], convexHull);

            double componentArea = cv::contourArea(contours[0]);
            double convexHullArea = cv::contourArea(convexHull);

            cv::RotatedRect boundingEllipse = cv::fitEllipse(contours[0]);
            float eccentricity = boundingEllipse.size.width / boundingEllipse.size.height;

            // cout << "width, height: " << boundingEllipse.size.width << ", " << boundingEllipse.size.height << endl;
            // cout << "eccentricity: " << eccentricity << endl;

            // cout << "componentArea, convexHullArea: " << componentArea << ", " << convexHullArea << endl;
            // cout << "ratio area: " << componentArea / convexHullArea << endl;

            if (eccentricity <= 0.6 || componentArea / convexHullArea < 0.6) // MODIFIER  POSSIBLEMENT ICI
            {
                filteredMask.setTo(255, componentMask);
            }
        }
        return filteredMask;
    }

    cv::Mat RobotSurgerySegmentation::maskOrgans(cv::Mat imagesrc, cv::Mat toolsMask)
    {
        cv::Mat inputImage = imagesrc.clone();

        int n = inputImage.rows;
        int m = inputImage.cols;
        int c = inputImage.channels();

        std::vector<float> mean = computeMean(inputImage);
        float meanH = mean[0];
        float meanS = mean[1];
        float meanV = mean[2];

        cv::cvtColor(inputImage, inputImage, cv::COLOR_BGR2HSV);

        torch::Tensor inputImageTensor = torch::from_blob(inputImage.data, {n, m, c}, torch::kByte).to(torch::kFloat32);
        inputImageTensor = inputImageTensor.permute({2, 0, 1}).contiguous();

        // Create meshgrid
        torch::Tensor xx = torch::arange(0, m);
        torch::Tensor yy = torch::arange(0, n);
        torch::Tensor xxTensor = xx.view({1, -1}).expand({n, -1});
        torch::Tensor yyTensor = yy.view({-1, 1}).expand({-1, m});
        xxTensor = xxTensor.unsqueeze(0);
        yyTensor = yyTensor.unsqueeze(0);

        torch::Tensor meanHTensor = torch::ones({1, n, m}) * meanH;
        torch::Tensor meanSTensor = torch::ones({1, n, m}) * meanS;
        torch::Tensor meanVTensor = torch::ones({1, n, m}) * meanV;

        torch::Tensor fuse = torch::cat({xxTensor, yyTensor, meanHTensor, meanSTensor, meanVTensor, inputImageTensor}, 0);
        fuse = fuse.permute({1, 2, 0}).contiguous().view({-1, 8});

        fuse = fuse.to(at::kCUDA);

        torch::NoGradGuard no_grad;
        torch::Tensor pred = modelOrgans.forward({fuse}).toTensor();
        pred = pred.to(at::kCPU);

        const float *dataPtr = pred.data_ptr<float>();
        int height = 270;
        int width = 480;
        cv::Mat predImage(height, width, CV_32F, const_cast<float *>(dataPtr));

        cv::Mat reoundedPredImage;
        cv::threshold(predImage, reoundedPredImage, 0.5, 255, cv::THRESH_BINARY);
        reoundedPredImage.convertTo(reoundedPredImage, CV_8U);

        cv::morphologyEx(reoundedPredImage, reoundedPredImage, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(15, 15)), cv::Point(-1, -1), 1);

        // MAIN

        cv::Mat closeToCenter = closestToCenterOfImageForEachArea(toolsMask);

        std::vector<cv::Mat> vec = reconstruct(reoundedPredImage, closeToCenter);
        cv::Mat reconstructed = vec[0];
        cv::Mat reconstructedWithout = vec[1];

        return reconstructed;
    }

    cv::Mat RobotSurgerySegmentation::mask(cv::Mat imagesrc)
    {
        cv::Mat image = imagesrc.clone();
        if (isBig)
        {
            cv::resize(image, image, cv::Size(1920, 1024));
        }
        else
        {
            image = image(cv::Range(7, 7 + 256), cv::Range(32, 32 + 416));
        }

        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        image.convertTo(image, CV_32FC3, 1.0f / 255.0f);

        torch::Tensor tensor_image = torch::from_blob(image.data, {1, image.rows, image.cols, 3});
        tensor_image = tensor_image.permute({0, 3, 1, 2});
        tensor_image = tensor_image.to(at::kCUDA);

        torch::NoGradGuard no_grad;
        torch::Tensor output = model.forward({tensor_image}).toTensor();

        output = output.to(at::kCPU);

        const float *dataPtr = output.data_ptr<float>();
        int height = output.size(2);
        int width = output.size(3);
        cv::Mat outputImage(height, width, CV_32F, const_cast<float *>(dataPtr));

        cv::threshold(outputImage, outputImage, 0.5, 255, cv::THRESH_BINARY);
        outputImage.convertTo(outputImage, CV_8U);
        // resize anyway
        cv::resize(outputImage, outputImage, cv::Size(480, 270));

        cv::Mat paddedMask = outputImage.clone();
        // cv::copyMakeBorder(outputImage, paddedMask, 7, 7, 32, 32, cv::BORDER_CONSTANT, 0);
        // juste pour tester on rempli le mask a gauche de 255 et de 0 a droite
        // paddedMask = cv::Mat::zeros(270, 480, CV_8UC1);
        // cv::rectangle(paddedMask, cv::Point(0, 0), cv::Point(240, 270), cv::Scalar(255), -1);
        // on elargit le masque de 10 pixels (dilate)

        cv::Mat paddedMaskMorphology = selectByMorphology(paddedMask, cv::Size2i(50, 5));
        cv::Mat paddedMaskEccentricity = selectByEccentricityAndConvexity(paddedMaskMorphology);

        cv::dilate(paddedMaskEccentricity, paddedMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10)));

        return paddedMask;
    }

    cv::Mat RobotSurgerySegmentation::fuse(cv::Mat frame, cv::Mat DLMask, cv::Mat DLMaskCenter, cv::Mat Reconstructed, cv::Mat NotReconstructed)
    {

        cv::Mat filter = cv::Mat::zeros(frame.size(), CV_8UC3);
        filter.setTo(cv::Scalar(255, 0, 0), NotReconstructed);
        filter.setTo(cv::Scalar(255, 255, 255), Reconstructed);
        filter.setTo(cv::Scalar(0, 255, 255), DLMask);
        filter.setTo(cv::Scalar(0, 255, 0), DLMaskCenter);

        cv::addWeighted(frame, 0.7, filter, 0.3, 0, filter);
        return filter;
    }

} // namespace ORB_SLAM2