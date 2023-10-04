#include "RobotSurgerySegmentation.h"

namespace ORB_SLAM2
{
    RobotSurgerySegmentation::RobotSurgerySegmentation(std::string modelPath, bool big)
    {
        std::string modelName = modelPath.substr(modelPath.find_last_of("/") + 1, modelPath.find_last_of(".") - modelPath.find_last_of("/") - 1);

        model = torch::jit::load(modelPath);
        model.to(at::kCUDA);
        isBig = big;
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