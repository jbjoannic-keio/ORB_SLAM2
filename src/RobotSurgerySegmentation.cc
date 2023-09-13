#include "RobotSurgerySegmentation.h"

namespace ORB_SLAM2
{
    RobotSurgerySegmentation::RobotSurgerySegmentation(std::string modelPath)
    {
        std::string modelName = modelPath.substr(modelPath.find_last_of("/") + 1, modelPath.find_last_of(".") - modelPath.find_last_of("/") - 1);
        std::cout << "Loading model: " << modelName << std::endl;
        model = torch::jit::load(modelPath);
        model.to(at::kCUDA);
    }

    cv::Mat RobotSurgerySegmentation::mask(cv::Mat image)
    {
        // TODO
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

        return outputImage;
    }

    cv::Mat RobotSurgerySegmentation::fuse(cv::Mat image, cv::Mat mask)
    {
        cv::Mat fused;
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        cv::addWeighted(image, 0.5, mask, 0.5, 0.0, fused);
        return fused;
    }

}