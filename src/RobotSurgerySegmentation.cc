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

    RobotSurgerySegmentation::RobotSurgerySegmentation(std::string modelPath, bool big, std::string modelOrgansPath)
    {
        std::string modelName = modelPath.substr(modelPath.find_last_of("/") + 1, modelPath.find_last_of(".") - modelPath.find_last_of("/") - 1);

        model = torch::jit::load(modelPath);
        modelOrgans = torch::jit::load(modelOrgansPath);
        model.to(at::kCUDA);
        modelOrgans.to(at::kCUDA);
        isBig = big;
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
            image = image(cv::Range(7, 7 + 256), cv::Range(32, 32 + 416));
            cv::resize(image, image, cv::Size(1664, 1024));
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
        cv::resize(outputImage, outputImage, cv::Size(416, 256));

        cv::Mat paddedMask;
        cv::copyMakeBorder(outputImage, paddedMask, 7, 7, 32, 32, cv::BORDER_CONSTANT, 0);
        // juste pour tester on rempli le mask a gauche de 255 et de 0 a droite
        // paddedMask = cv::Mat::zeros(270, 480, CV_8UC1);
        // cv::rectangle(paddedMask, cv::Point(0, 0), cv::Point(240, 270), cv::Scalar(255), -1);
        // on elargit le masque de 10 pixels (dilate)
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
        cv::dilate(paddedMask, paddedMask, element);

        return paddedMask;
    }

    cv::Mat RobotSurgerySegmentation::fuse(cv::Mat image, cv::Mat mask)
    {
        cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
        cv::Mat fused;

        cv::addWeighted(image, 0.5, mask, 0.5, 0.0, fused);
        return fused;
    }

} // namespace ORB_SLAM2