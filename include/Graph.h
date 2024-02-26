#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
    struct Node
    {
        int id;
        cv::Point coordinates;
        std::unordered_map<int, double> edges;
        int type; // 0 = branch //1 = tip //2 = junction
    };

    struct Blob
    {
        std::unordered_map<int, Node> nodes;
    };

    class Graph
    {
    private:
        std::unordered_map<int, Blob> blobs;
        std::unordered_map<int, std::pair<double, std::vector<int>>> longestPathPerBlob;

    public:
        void addNode(int id, cv::Point coordinates, int type = 0, int blobId = 0);
        double computeDistanceAlongContour(const std::vector<std::vector<std::vector<cv::Point>>> &simplifiedContour, const cv::Point &p1, const cv::Point &p2);
        double computeArcLength(cv::Mat &skeletonImage, const cv::Point &p1, const cv::Point &p2);
        double distance(const cv::Point &p1, const cv::Point &p2);
        std::vector<std::pair<cv::Point, int>> findBranchPoints(cv::Mat &skeleton);
        void createGraphFromSkeleton(cv::Mat &skeletonImage);
        void addEdge(int from, int to, double distance, int blobId);
        void displayGraph();
        void showGraph(cv::Mat &skeletonImage);
        void showGraphSimple(cv::Mat &skeletonImage);
        std::vector<std::vector<std::vector<cv::Point>>> simplifyContour(const std::vector<cv::Point> &contour, cv::Point p1, cv::Point p2);
        double getShortestPathDistance(int idNode1, int idNode2, int blobId);
        std::pair<double, std::vector<int>> getShortestPathDistanceAndPath(int idNode1, int idNode2, int blobId);
        std::unordered_map<int, std::vector<int>> reallyConnected(cv::Mat &skeletonImage);
        int findPolygon(cv::Mat &skeletonImage, const cv::Point p1);
        std::unordered_map<int, std::pair<double, std::vector<int>>> getLongestPathPerBlob();
        std::vector<std::vector<cv::Point>> getAllBlobImage(cv::Mat &skeletonImage, int blobId);
        std::vector<std::vector<cv::Point>> getLongestPathPerBlobImage(cv::Mat &skeletonImage, int blobId, std::vector<int> path);
        std::pair<cv::Mat, std::vector<cv::Point>> getLongestSkeleton(cv::Mat &skeletonImage, double threshold);
        cv::Point getSAMPointFromSkelet(cv::Mat &skeletonImage, int blobId);
        cv::Point getSAMPointFromSkeletAllPaths(cv::Mat &skeletonImage, int blobId);
        cv::Point getMeanCoord(int blobId);
    };
} // namespace ORB_SLAM2

#endif // GRAPH_H