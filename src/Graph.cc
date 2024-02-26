#include <Graph.h>

namespace ORB_SLAM2
{

    int Graph::findPolygon(cv::Mat &skeletonImage, const cv::Point p1)
    {

        return -1;
    };

    void Graph::addNode(int id, cv::Point coordinates, int type, int blobId)
    {
        if (blobs.find(blobId) == blobs.end())
        {
            // std::cout << "Creating new blob with ID " << blobId << std::endl;
            Blob newBlob;
            blobs[blobId] = newBlob;
        }
        std::unordered_map<int, Node> nodes = blobs[blobId].nodes;
        bool alreadyExists = false;
        int oldId = -1;
        if (nodes.find(id) != nodes.end())
        {
            alreadyExists = true;
        }
        for (auto &nodePair : nodes)
        {
            if (nodePair.second.coordinates == coordinates)
            {
                alreadyExists = true;
                oldId = nodePair.first;
                break;
            }
        }
        if (!alreadyExists)
        {
            Node newNode;
            newNode.id = id;
            newNode.coordinates = coordinates;
            newNode.type = type;
            nodes[id] = newNode;
            blobs[blobId].nodes = nodes;
        }
        else
        {
            if (type == 2)
            {
                if (nodes[id].type != 2)
                {
                    nodes[oldId].type = 2;
                    blobs[blobId].nodes = nodes;
                }
            }
            // std::cout << "Node with ID " << id << " already exists" << std::endl;
        }
    };

    double Graph::computeDistanceAlongContour(const std::vector<std::vector<std::vector<cv::Point>>> &simplifiedContour, const cv::Point &p1, const cv::Point &p2)
    {
        double distanceMin = std::numeric_limits<double>::max();

        for (int i = 0; i < simplifiedContour.size(); ++i)
        {
            for (int j = 0; j < simplifiedContour[i].size(); ++j)
            {
                // on a le contouur simplifié
                double distance = 0.0;
                // // std::cout << "i = " << i << " j = " << j << " size " << simplifiedContour[i][j].size() << std::endl;
                for (int k = 0; k < simplifiedContour[i][j].size() - 1; ++k)
                {
                    // // std::cout << "k = " << k << std::endl;
                    // // std::cout << "d " << cv::norm(simplifiedContour[i][j][k] - simplifiedContour[i][j][k + 1]) << std::endl;
                    distance += cv::norm(simplifiedContour[i][j][k] - simplifiedContour[i][j][k + 1]);
                }
                // // std::cout << "Distance = " << distance << std::endl;
                distanceMin = std::min(distanceMin, distance);
            }
        }
        return distanceMin;
    };

    std::vector<std::vector<std::vector<cv::Point>>> Graph::simplifyContour(const std::vector<cv::Point> &contour, cv::Point p1, cv::Point p2)
    {
        // search for p1 and p2 in the contour
        std::vector<int> positionsP1;
        std::vector<int> positionsP2;
        for (int i = 0; i < contour.size(); ++i)
        {
            if (contour[i] == p1)
            {
                positionsP1.push_back(i);
            }
            if (contour[i] == p2)
            {
                positionsP2.push_back(i);
            }
        }

        std::vector<std::vector<std::vector<cv::Point>>> result;

        for (auto i1 : positionsP1)
        {
            std::vector<cv::Point> shiftContourTrigo;
            std::vector<cv::Point> shiftContourAntiTrigo;
            for (int i = i1; i < contour.size(); ++i)
            {
                shiftContourTrigo.push_back(contour[i]);
            }
            for (int i = 0; i < i1; ++i)
            {
                shiftContourTrigo.push_back(contour[i]);
            }
            for (int i = i1; i >= 0; --i)
            {
                shiftContourAntiTrigo.push_back(contour[i]);
            }
            for (int i = contour.size() - 1; i > i1; --i)
            {
                shiftContourAntiTrigo.push_back(contour[i]);
            }
            for (auto i2 : positionsP2)
            {
                // std::cout << "i1 = " << i1 << " i2 = " << i2 << std::endl;
                // std::cout << "shiftContourTrigo" << std::endl;
                for (int i = 0; i < shiftContourTrigo.size(); ++i)
                {
                    // std::cout << shiftContourTrigo[i] << "  ";
                }
                // std::cout << std::endl;
                // std::cout << "shiftContourAntiTrigo" << std::endl;
                for (int i = 0; i < shiftContourAntiTrigo.size(); ++i)
                {
                    // std::cout << shiftContourAntiTrigo[i] << "  ";
                }
                // std::cout << std::endl;
                if (std::norm(i2 - i1) <= 1)
                {
                    result.push_back({contour});
                }
                else
                {
                    std::vector<cv::Point> simplifiedContourTrigo;
                    std::vector<cv::Point> simplifiedContourAntiTrigo;
                    std::vector<int> simplifiedContourTrigoIndex;
                    std::vector<int> simplifiedContourAntiTrigoIndex;
                    int newI2Trigo, newI2AntiTrigo;
                    if (i2 > i1)
                    {
                        newI2Trigo = i2 - i1;
                        newI2AntiTrigo = i1 - i2 + contour.size();
                    }
                    else
                    {
                        newI2Trigo = i2 - i1 + contour.size();
                        newI2AntiTrigo = i1 - i2;
                    }
                    // std::cout << "newI2Trigo = " << newI2Trigo << " newI2AntiTrigo = " << newI2AntiTrigo << std::endl;
                    std::vector<int> eraseTrigoStart;
                    std::vector<int> eraseTrigoEnd;
                    std::vector<int> eraseAntiTrigoStart;
                    std::vector<int> eraseAntiTrigoEnd;

                    for (int i = 1; i < newI2Trigo; ++i)
                    {
                        cv::Point currentPoint = shiftContourTrigo[i];
                        // std::cout << "i" << i << "currentPoint " << currentPoint << std::endl;
                        for (int j = i + 1; j < newI2Trigo; j++)
                        {
                            // std::cout << "j" << j << " and " << shiftContourTrigo[j] << std::endl;
                            if (shiftContourTrigo[j] == currentPoint)
                            {
                                // std::cout << "Found double TRIGO"
                                // << " i " << i << " j " << j << std::endl;

                                eraseTrigoStart.push_back(i);
                                eraseTrigoEnd.push_back(j);
                                i = j;

                                break;
                            }
                        }
                    }
                    for (int i = 1; i < newI2AntiTrigo; ++i)
                    {
                        cv::Point currentPoint = shiftContourAntiTrigo[i];

                        for (int j = i + 1; j < newI2AntiTrigo; j++)
                        {
                            if (shiftContourAntiTrigo[j] == currentPoint)
                            {
                                // std::cout << "Found double ANTITRIGO"
                                // << " i " << i << " j " << j << std::endl;
                                eraseAntiTrigoStart.push_back(i);
                                eraseAntiTrigoEnd.push_back(j);
                                i = j;
                                break;
                            }
                        }
                    }
                    for (int i = 0; i < eraseTrigoStart.size(); ++i)
                    {
                        // std::cout << "remove from" << eraseTrigoStart[i] << " to " << eraseTrigoEnd[i] << std::endl;
                    }
                    for (int i = 0; i <= newI2Trigo; ++i)
                    {
                        bool shouldCopy = true;
                        for (int j = 0; j < eraseTrigoStart.size(); ++j)
                        {
                            if (i > eraseTrigoStart[j] && i <= eraseTrigoEnd[j])
                            {
                                shouldCopy = false;
                                break;
                            }
                        }
                        if (shouldCopy)
                            simplifiedContourTrigo.push_back(shiftContourTrigo[i]);
                    }
                    // std::cout << "antiTrigo" << newI2AntiTrigo << std::endl;
                    for (int i = 0; i < eraseAntiTrigoStart.size(); ++i)
                    {
                        // std::cout << "remove from" << eraseAntiTrigoStart[i] << " to " << eraseAntiTrigoEnd[i] << std::endl;
                    }
                    for (int i = 0; i <= newI2AntiTrigo; ++i)
                    {
                        bool shouldCopy = true;
                        for (int j = 0; j < eraseAntiTrigoStart.size(); ++j)
                        {
                            if (i > eraseAntiTrigoStart[j] && i <= eraseAntiTrigoEnd[j])
                            {
                                shouldCopy = false;
                                break;
                            }
                        }
                        if (shouldCopy)
                            simplifiedContourAntiTrigo.push_back(shiftContourAntiTrigo[i]);
                    }
                    // std::cout << simplifiedContourTrigo.size() << " " << simplifiedContourAntiTrigo.size() << std::endl;
                    result.push_back({simplifiedContourTrigo, simplifiedContourAntiTrigo});
                }
            }
        }
        return result;
    };
    double Graph::computeArcLength(cv::Mat &skeletonImage, const cv::Point &p1, const cv::Point &p2)
    {
        // std::cout << "Computing arc length between " << p1 << " and " << p2 << std::endl;
        cv::Mat visualization = skeletonImage.clone();
        cv::cvtColor(visualization, visualization, cv::COLOR_GRAY2BGR);
        cv::circle(visualization, p1, 2, cv::Scalar(255, 0, 0), -1);
        cv::circle(visualization, p2, 2, cv::Scalar(255, 0, 0), -1);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(skeletonImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        double minArcLength = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto &contour : contours)
        {
            if (cv::pointPolygonTest(contour, p1, false) == 0 && cv::pointPolygonTest(contour, p2, false) == 0)
            {
                // std::cout << "Contour found" << std::endl;
                found = true;
                for (int i = 0; i < contour.size(); ++i)
                {
                    // std::cout << contour[i] << "  ";
                    cv::circle(visualization, contour[i], 1, cv::Scalar(0, 0, 255), -1);
                }
                // std::cout << std::endl;
                std::vector<std::vector<std::vector<cv::Point>>> simplifiedContour;

                simplifiedContour = simplifyContour(contour, p1, p2);
                // std::cout << "Simplified contour: " << std::endl;S

                for (int i = 0; i < simplifiedContour.size(); ++i)
                {
                    for (int j = 0; j < simplifiedContour[i].size(); ++j)
                    {
                        // std::cout << "Contour " << i << " " << j << " : " << std::endl;
                        for (int k = 0; k < simplifiedContour[i][j].size(); ++k)
                        {
                            // std::cout << simplifiedContour[i][j][k] << "  ";
                            cv::circle(visualization, simplifiedContour[i][j][k], 1, cv::Scalar(0, 255, 0), -1);
                        }
                        // std::cout << std::endl;
                    }
                }

                // std::cout << std::endl;
                double distance = computeDistanceAlongContour(simplifiedContour, p1, p2);
                minArcLength = std::min(minArcLength, distance);
            }
        }
        if (minArcLength == std::numeric_limits<double>::max())
            return 0.0;
        return minArcLength;
    };

    double Graph::distance(const cv::Point &p1, const cv::Point &p2)
    {
        return cv::norm(p1 - p2);
    };

    std::vector<std::pair<cv::Point, int>> Graph::findBranchPoints(cv::Mat &skeleton)
    {
        std::vector<std::pair<cv::Point, int>> branchPoints;

        for (int y = 1; y < skeleton.rows - 1; ++y)
        {
            for (int x = 1; x < skeleton.cols - 1; ++x)
            {
                if (skeleton.at<uchar>(y, x) == 0)
                {
                    continue;
                }
                cv::Mat neighnorhood = skeleton(cv::Rect(x - 1, y - 1, 3, 3));

                int count = cv::countNonZero(neighnorhood);
                if (count >= 5) // branch
                {
                    bool doTransfer = false;
                    if (skeleton.at<uchar>(y - 1, x) != 0)
                    {
                        doTransfer = true;
                        branchPoints.push_back(std::make_pair(cv::Point(x, y - 1), 0));
                    }
                    if (skeleton.at<uchar>(y + 1, x) != 0)
                    {
                        doTransfer = true;
                        branchPoints.push_back(std::make_pair(cv::Point(x, y + 1), 0));
                    }
                    if (skeleton.at<uchar>(y, x + 1) != 0)
                    {
                        doTransfer = true;
                        branchPoints.push_back(std::make_pair(cv::Point(x + 1, y), 0));
                    }
                    if (skeleton.at<uchar>(y, x - 1) != 0)
                    {
                        doTransfer = true;
                        branchPoints.push_back(std::make_pair(cv::Point(x - 1, y), 0));
                    }
                    if (skeleton.at<uchar>(y - 1, x - 1) != 0)
                    {
                        doTransfer = true;
                        branchPoints.push_back(std::make_pair(cv::Point(x - 1, y - 1), 0));
                    }
                    if (skeleton.at<uchar>(y - 1, x + 1) != 0)
                    {
                        doTransfer = true;
                        branchPoints.push_back(std::make_pair(cv::Point(x + 1, y - 1), 0));
                    }
                    if (skeleton.at<uchar>(y + 1, x - 1) != 0)
                    {
                        doTransfer = true;
                        branchPoints.push_back(std::make_pair(cv::Point(x - 1, y + 1), 0));
                    }
                    if (skeleton.at<uchar>(y + 1, x + 1) != 0)
                    {
                        doTransfer = true;
                        branchPoints.push_back(std::make_pair(cv::Point(x + 1, y + 1), 0));
                    }
                    if (doTransfer)
                    {
                        branchPoints.push_back(std::make_pair(cv::Point(x, y), 2));
                    }
                }
                if (count == 4) // branch
                {
                    // We need to test if the neighbour are connected
                    std::vector<int> dx = {-1, -1, -1, 0, 1, 1, 1, 0, -1};
                    std::vector<int> dy = {-1, 0, 1, 1, 1, 0, -1, -1, -1};
                    bool connected = false;
                    for (int i = 0; i < dx.size() - 1; ++i)
                    {
                        if (skeleton.at<uchar>(y + dy[i], x + dx[i]) != 0 && skeleton.at<uchar>(y + dy[i + 1], x + dx[i + 1]) != 0)
                        {
                            connected = true;
                            break;
                        }
                    }
                    if (!connected)
                    {
                        bool doTransfer = false;
                        if (skeleton.at<uchar>(y - 1, x) != 0)
                        {
                            doTransfer = true;
                            branchPoints.push_back(std::make_pair(cv::Point(x, y - 1), 0));
                        }
                        if (skeleton.at<uchar>(y + 1, x) != 0)
                        {
                            doTransfer = true;
                            branchPoints.push_back(std::make_pair(cv::Point(x, y + 1), 0));
                        }
                        if (skeleton.at<uchar>(y, x + 1) != 0)
                        {
                            doTransfer = true;
                            branchPoints.push_back(std::make_pair(cv::Point(x + 1, y), 0));
                        }
                        if (skeleton.at<uchar>(y, x - 1) != 0)
                        {
                            doTransfer = true;
                            branchPoints.push_back(std::make_pair(cv::Point(x - 1, y), 0));
                        }
                        if (skeleton.at<uchar>(y - 1, x - 1) != 0)
                        {
                            doTransfer = true;
                            branchPoints.push_back(std::make_pair(cv::Point(x - 1, y - 1), 0));
                        }
                        if (skeleton.at<uchar>(y - 1, x + 1) != 0)
                        {
                            doTransfer = true;
                            branchPoints.push_back(std::make_pair(cv::Point(x + 1, y - 1), 0));
                        }
                        if (skeleton.at<uchar>(y + 1, x - 1) != 0)
                        {
                            doTransfer = true;
                            branchPoints.push_back(std::make_pair(cv::Point(x - 1, y + 1), 0));
                        }
                        if (skeleton.at<uchar>(y + 1, x + 1) != 0)
                        {
                            doTransfer = true;
                            branchPoints.push_back(std::make_pair(cv::Point(x + 1, y + 1), 0));
                        }
                        if (doTransfer)
                        {
                            branchPoints.push_back(std::make_pair(cv::Point(x, y), 2));
                        }
                    }
                }
                if (count == 3) // tip only if connected
                {
                    std::vector<int> dx = {-1, -1, -1, 0, 1, 1, 1, 0, -1};
                    std::vector<int> dy = {-1, 0, 1, 1, 1, 0, -1, -1, -1};
                    bool connected = false;
                    for (int i = 0; i < dx.size() - 1; ++i)
                    {
                        if (skeleton.at<uchar>(y + dy[i], x + dx[i]) != 0 && skeleton.at<uchar>(y + dy[i + 1], x + dx[i + 1]) != 0)
                        {
                            connected = true;
                            break;
                        }
                    }
                    if (connected)
                        branchPoints.push_back(std::make_pair(cv::Point(x, y), 1));
                }
                if (count == 2) // tip
                {
                    branchPoints.push_back(std::make_pair(cv::Point(x, y), 1));
                }
            }
        }

        return branchPoints;
    };

    double Graph::getShortestPathDistance(int idNode1, int idNode2, int blobId)
    {
        if (blobs.find(blobId) == blobs.end())
        {
            std::cerr << "Blob not found in the graph!" << std::endl;
            return -1; // Return a flag or error value indicating failure
        }
        std::unordered_map<int, Node> nodes = blobs[blobId].nodes;
        if (nodes.find(idNode1) == nodes.end() || nodes.find(idNode2) == nodes.end())
        {
            std::cerr << "One or both nodes not found in the graph!" << std::endl;
            return -1; // Return a flag or error value indicating failure
        }

        // Dijkstra's algorithm implementation
        std::unordered_map<int, double> dist; // Store shortest distances from source
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

        for (const auto &pair : nodes)
        {
            dist[pair.first] = std::numeric_limits<double>::max(); // Initialize distances to infinity
        }

        dist[idNode1] = 0; // Distance from source to itself is 0
        pq.push({0, idNode1});

        while (!pq.empty())
        {
            int u = pq.top().second;
            pq.pop();

            for (const auto &edge : nodes[u].edges)
            {
                int v = edge.first;
                double weight = edge.second;

                if (dist[v] > dist[u] + weight)
                {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        return dist[idNode2]; // Return shortest distance between idNode1 and idNode2
    };

    std::pair<double, std::vector<int>> Graph::getShortestPathDistanceAndPath(int idNode1, int idNode2, int blobId)
    {
        if (blobs.find(blobId) == blobs.end())
        {
            std::cerr << "Blob not found in the graph!" << std::endl;
            std::vector<int> empty;
            return std::make_pair(-1, empty); // Return a flag or error value indicating failure
        }
        std::unordered_map<int, Node> nodes = blobs[blobId].nodes;
        if (nodes.find(idNode1) == nodes.end() || nodes.find(idNode2) == nodes.end())
        {
            std::cerr << "One or both nodes not found in the graph!" << std::endl;
            std::vector<int> empty;
            return std::make_pair(-1, empty); // Return a flag or error value indicating failure
        }

        // Dijkstra's algorithm implementation
        std::unordered_map<int, double> dist; // Store shortest distances from source
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

        for (const auto &pair : nodes)
        {
            dist[pair.first] = std::numeric_limits<double>::max(); // Initialize distances to infinity
        }

        dist[idNode1] = 0; // Distance from source to itself is 0
        pq.push({0, idNode1});

        std::unordered_map<int, int> prevNode;

        // std::cout << "Dijkstra's algorithm" << std::endl;
        // std::cout << idNode1 << " " << idNode2 << std::endl;

        while (!pq.empty())
        {
            int u = pq.top().second;
            pq.pop();

            for (const auto &edge : nodes[u].edges)
            {
                int v = edge.first;
                double weight = edge.second;

                if (dist[v] > dist[u] + weight)
                {
                    dist[v] = dist[u] + weight;
                    prevNode[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        // std::cout << "end first loop" << std::endl;

        int currentNode = idNode2;
        std::vector<int> path;

        if (prevNode.find(idNode2) == prevNode.end())
        {
            std::vector<int> empty;
            // std::cout << "No path found" << std::endl;
            return std::make_pair(-1, empty); // Return a flag or error value indicating failure
        }
        while (currentNode != idNode1)
        {
            // std::cout << "currentNode " << currentNode << std::endl;
            path.push_back(currentNode);
            currentNode = prevNode[currentNode];
        }

        // std::cout << "end second loop" << std::endl;
        path.push_back(idNode1);

        return std::make_pair(dist[idNode2], path); // Return shortest distance between idNode1 and idNode2
    };

    std::unordered_map<int, std::vector<int>> Graph::reallyConnected(cv::Mat &skeletonImage)
    {
        std::unordered_map<int, std::vector<int>> connections;
        for (auto &blobPair : blobs)
        {
            std::unordered_map<int, Node> nodes = blobPair.second.nodes;
            for (auto &nodePair1 : nodes)
            {
                // std::cout << "Node " << nodePair1.first << std::endl;
                std::vector<int> connection;
                if (nodePair1.second.type == 2)
                {
                    // std::cout << "Junction" << std::endl;
                    continue;
                }
                for (auto &nodePair2 : nodes)
                {
                    if (nodePair2.second.type == 2)
                    {
                        // std::cout << "Junction" << std::endl;
                        continue;
                    }
                    // std::cout << "      With Node " << nodePair2.first << std::endl;
                    if (nodePair1.first >= nodePair2.first)
                    {
                        continue;
                    }
                    cv::Mat skeletWithoutOther = skeletonImage.clone();

                    for (auto &nodePair3 : nodes)
                    {
                        if (nodePair3.first == nodePair1.first || nodePair3.first == nodePair2.first)
                        {
                            continue;
                        }
                        // put a square around the node
                        if (nodePair3.second.type != 2)
                            skeletWithoutOther.at<uchar>(nodePair3.second.coordinates) = 0;
                    }

                    std::vector<std::vector<cv::Point>> contours;
                    cv::findContours(skeletWithoutOther, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                    for (auto &contour : contours)
                    {
                        if (cv::pointPolygonTest(contour, nodePair1.second.coordinates, false) >= 0 && cv::pointPolygonTest(contour, nodePair2.second.coordinates, false) >= 0)
                        {
                            // std::cout << "Node " << nodePair1.first << " is connected to " << nodePair2.first << std::endl;
                            connection.push_back(nodePair2.first);
                        }
                    }
                }
                connections[nodePair1.first] = connection;
            }
        }

        return connections;
    }

    void Graph::createGraphFromSkeleton(cv::Mat &skeletonImage)
    {
        blobs.clear();

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(skeletonImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> filteredContours;
        for (int i = 0; i < contours.size(); ++i)
        {
            if (hierarchy[i][3] == -1)
            {
                filteredContours.push_back(contours[i]);
            }
        }
        skeletonImage = cv::Mat::zeros(skeletonImage.size(), CV_8UC1);
        for (int i = 0; i < filteredContours.size(); ++i)
        {
            cv::drawContours(skeletonImage, filteredContours, i, cv::Scalar(255), 1);
        }
        std::vector<std::pair<cv::Point, int>> branchPoints = findBranchPoints(skeletonImage);
        cv::Mat labels;
        int numLabels = cv::connectedComponents(skeletonImage, labels, 8, CV_32S);
        // add nodes
        for (int i = 0; i < branchPoints.size(); ++i)
        {
            // std::cout << "Adding node " << i << " with coordinates " << branchPoints[i].first << "in blob " << labels.at<int>(branchPoints[i].first) << " is tip " << branchPoints[i].second << std::endl;
            addNode(i, branchPoints[i].first, branchPoints[i].second, labels.at<int>(branchPoints[i].first));
        }

        std::unordered_map<int, std::vector<int>> connections = reallyConnected(skeletonImage);

        // add edges

        for (auto &blobPair : blobs)
        {
            for (auto &nodePair : blobPair.second.nodes)
            {
                for (auto &connection : connections[nodePair.first])
                {
                    // std::cout << std::endl;
                    // std::cout << "______________________________________" << std::endl;
                    // std::cout << "Computing distance between " << nodePair.first << " and " << connection << std::endl;
                    cv::Mat skeletWithoutOther = skeletonImage.clone();
                    for (auto &nodePair3 : blobPair.second.nodes)
                    {
                        if (nodePair3.first == nodePair.first || nodePair3.first == connection)
                        {
                            continue;
                        }
                        // put a square around the node
                        if (nodePair3.second.type != 2)
                            skeletWithoutOther.at<uchar>(nodePair3.second.coordinates) = 0;
                        // cv::rectangle(skeletWithoutOther, cv::Point(nodePair3.second.coordinates.x - 1, nodePair3.second.coordinates.y - 1), cv::Point(nodePair3.second.coordinates.x + 1, nodePair3.second.coordinates.y + 1), cv::Scalar(0), -1);
                    }
                    double dist = computeArcLength(skeletWithoutOther, nodePair.second.coordinates, blobPair.second.nodes[connection].coordinates);
                    // std::cout << "Node " << nodePair.first << " is connected to " << connection << " with distance " << dist << std::endl;
                    if (dist != 0.0)
                    {
                        // std::cout << "Node " << nodePair.first << " is connected to " << connection << " with distance " << dist << std::endl;
                        addEdge(nodePair.first, connection, dist, blobPair.first);
                    }
                }
            }
        }
    };

    void Graph::addEdge(int from, int to, double distance, int blobId)
    {
        if (blobs.find(blobId) != blobs.end())
        {
            if (blobs[blobId].nodes.find(from) != blobs[blobId].nodes.end() && blobs[blobId].nodes.find(to) != blobs[blobId].nodes.end())
            {
                blobs[blobId].nodes[from].edges[to] = distance;
                blobs[blobId].nodes[to].edges[from] = distance; // undirected graph
            }
            else
            {
                // std::cout << "Node with ID " << from << " or " << to << " does not exist" << std::endl;
            }
        }
        else
        {
            // std::cout << "Blob with ID " << blobId << " does not exist" << std::endl;
        }
    };

    void Graph::displayGraph()
    {
        for (const auto &pairBlob : blobs)
        {
            // std::cout << "BLOB " << pairBlob.first << std::endl;
            for (const auto &pairNodes : pairBlob.second.nodes)
            {
                // std::cout << "Node" << pairNodes.second.id << "is connected to:" << std::endl;
                for (const auto &edge : pairNodes.second.edges)
                {
                    // std::cout << " -> Node " << edge.first << " with distance : " << edge.second << std::endl;
                }
                // std::cout << std::endl;
            }
            // std::cout << std::endl;
        }
    };

    cv::Point Graph::getMeanCoord(int blobId)
    {
        cv::Point meanCoord;
        int count = 0;
        for (const auto &pairNodes : blobs[blobId].nodes)
        {
            meanCoord += pairNodes.second.coordinates;
            count++;
        }
        meanCoord /= count;
        return meanCoord;
    };

    std::unordered_map<int, std::pair<double, std::vector<int>>> Graph::getLongestPathPerBlob()
    {
        // std::cout << "Computing longest path per blob" << std::endl;
        std::unordered_map<int, std::pair<double, std::vector<int>>> result;
        for (auto &blobPair : blobs)
        {
            // std::cout << "Blob " << blobPair.first << std::endl;
            double maxDistance = 0.0;
            std::vector<int> longestPath;
            for (auto &nodePair1 : blobPair.second.nodes)
            {
                if (nodePair1.second.type != 1)
                {
                    continue;
                }
                else
                {
                    for (auto &nodePair2 : blobPair.second.nodes)
                    {
                        if (nodePair1.first == nodePair2.first || nodePair2.second.type != 1)
                        {
                            continue;
                        }
                        else
                        {
                            std::pair<double, std::vector<int>> path = getShortestPathDistanceAndPath(nodePair1.first, nodePair2.first, blobPair.first);
                            // double distance = getShortestPathDistance(nodePair1.first, nodePair2.first, blobPair.first);
                            double distance = path.first;
                            if (distance > maxDistance)
                            {
                                maxDistance = distance;
                                longestPath = path.second;
                                result[blobPair.first] = std::make_pair(maxDistance, longestPath);
                                // longestPath = {nodePair1.first, nodePair2.first};
                            }
                        }
                    }
                }
            }
        }
        for (auto &pair : result)
        {
            // std::cout << "Blob " << pair.first << " longest path: " << std::endl;
            for (auto &id : pair.second.second)
            {
                // std::cout << id << " ";
            }
            // std::cout << " with distance " << pair.second.first << std::endl;
            // std::cout << std::endl;
        }
        longestPathPerBlob = result;
        return result;
    }

    std::vector<std::vector<cv::Point>> Graph::getAllBlobImage(cv::Mat &skeletonImage, int blobId)
    {
        cv::Mat skeletonWithoutOther = skeletonImage.clone();
        std::vector<std::vector<cv::Point>> outliersContours, contours;
        cv::findContours(skeletonWithoutOther, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto &nodePair : blobs[blobId].nodes)
        {
            for (auto &contour : contours)
            {
                if (cv::pointPolygonTest(contour, nodePair.second.coordinates, false) >= 0)
                {
                    // std::cout << "Contour with outlier found" << std::endl;

                    outliersContours.push_back(contour);
                }
            }
        }
        return outliersContours;
    }

    std::vector<std::vector<cv::Point>> Graph::getLongestPathPerBlobImage(cv::Mat &skeletonImage, int blobId, std::vector<int> path)
    {
        cv::Mat skeletonWithoutOther = skeletonImage.clone();
        std::vector<std::vector<cv::Point>> outliersContours;
        for (auto &nodePair : blobs[blobId].nodes)
        {
            if (std::find(path.begin(), path.end(), nodePair.first) != path.end())
            {
                skeletonWithoutOther.at<uchar>(nodePair.second.coordinates) = 0;
                // cv::rectangle(skeletonWithoutOther, cv::Point(nodePair.second.coordinates.x - 1, nodePair.second.coordinates.y - 1), cv::Point(nodePair.second.coordinates.x + 1, nodePair.second.coordinates.y + 1), cv::Scalar(0), -1);
            }
            else if (nodePair.second.type == 2)
            {
                bool isNeighbor = false;
                for (auto &nodePath : path)
                {
                    if (std::abs(nodePair.second.coordinates.x - blobs[blobId].nodes[nodePath].coordinates.x) <= 2 && std::abs(nodePair.second.coordinates.y - blobs[blobId].nodes[nodePath].coordinates.y) <= 2)
                    {
                        isNeighbor = true;
                        skeletonWithoutOther.at<uchar>(nodePair.second.coordinates) = 0;
                        break;
                    }
                }
            }
        }
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(skeletonWithoutOther, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto &nodePair : blobs[blobId].nodes)
        {
            if (std::find(path.begin(), path.end(), nodePair.first) == path.end())
            {
                for (auto &contour : contours)
                {
                    if (cv::pointPolygonTest(contour, nodePair.second.coordinates, false) >= 0)
                    {
                        // std::cout << "Contour with outlier found" << std::endl;

                        outliersContours.push_back(contour);
                    }
                }
            }
        }

        return outliersContours;
    }
    std::pair<cv::Mat, std::vector<cv::Point>> Graph::getLongestSkeleton(cv::Mat &skeletonImage, double threshold)
    {
        // std::cout << "Computing longest skeleton" << std::endl;
        std::unordered_map<int, std::pair<double, std::vector<int>>> longestPathPerBlob = getLongestPathPerBlob();
        cv::Mat result = cv::Mat::zeros(skeletonImage.size(), CV_8UC1);
        cv::Mat soustraction = skeletonImage.clone();

        std::vector<cv::Point> extremitiesForSAM;

        for (auto &pair : longestPathPerBlob)
        {
            std::vector<std::vector<cv::Point>> outlierContours = getLongestPathPerBlobImage(skeletonImage, pair.first, pair.second.second);
            int i = 0;
            if (pair.second.first > threshold)
            {
                // std::cout << "BlobThreshold " << pair.first << " longest path: " << std::endl;
                cv::Point extremity = getSAMPointFromSkelet(skeletonImage, pair.first);
                cv::Point extremity2 = getSAMPointFromSkeletAllPaths(skeletonImage, pair.first);
                // std::cout << "Extremity " << extremity << std::endl;
                extremitiesForSAM.push_back(extremity2);
                for (auto &contour : outlierContours)
                {
                    cv::Mat prov = cv::Mat::zeros(skeletonImage.size(), CV_8UC1);
                    // cv::polylines(result, contour, false, cv::Scalar(255), 1);
                    // cv::drawContours(result, {contour}, -1, cv::Scalar(255), -1);
                    cv::drawContours(prov, outlierContours, i, cv::Scalar(255), cv::FILLED);
                    result += prov;
                    i++;
                }
            }
            else
            {
                std::vector<std::vector<cv::Point>> allBlobContours = getAllBlobImage(skeletonImage, pair.first);
                for (auto &contour : allBlobContours)
                {
                    cv::Mat prov = cv::Mat::zeros(skeletonImage.size(), CV_8UC1);
                    // cv::polylines(result, contour, false, cv::Scalar(255), 1);
                    // cv::drawContours(result, {contour}, -1, cv::Scalar(255), -1);
                    cv::drawContours(prov, allBlobContours, i, cv::Scalar(255), cv::FILLED);
                    result += prov;
                    i++;
                }
            }
        }
        soustraction -= result;
        std::pair<cv::Mat, std::vector<cv::Point>> resultPair = std::make_pair(soustraction, extremitiesForSAM);
        return resultPair;
    }

    cv::Point Graph::getSAMPointFromSkelet(cv::Mat &skeletonImage, int blobId)
    {
        cv::Point result;
        cv::Point2d pointExtreme;
        for (auto &pair : longestPathPerBlob)
        {
            if (pair.first == blobId)
            {
                std::vector<cv::Point> points;
                Node nodeExtremity1 = blobs[blobId].nodes[pair.second.second[0]];
                Node nodeExtremity2 = blobs[blobId].nodes[pair.second.second[pair.second.second.size() - 1]];
                // distance from borders
                double distance1 = std::min(std::min(nodeExtremity1.coordinates.x - 30, nodeExtremity1.coordinates.y), std::min(skeletonImage.cols - 30 - nodeExtremity1.coordinates.x, skeletonImage.rows - nodeExtremity1.coordinates.y));
                double distance2 = std::min(std::min(nodeExtremity2.coordinates.x - 30, nodeExtremity2.coordinates.y), std::min(skeletonImage.cols - 30 - nodeExtremity2.coordinates.x, skeletonImage.rows - nodeExtremity2.coordinates.y));
                Node neighbour;
                if (distance1 > distance2)
                {
                    result = nodeExtremity1.coordinates;
                    neighbour = blobs[blobId].nodes[pair.second.second[1]];
                }
                else
                {
                    result = nodeExtremity2.coordinates;
                    neighbour = blobs[blobId].nodes[pair.second.second[pair.second.second.size() - 2]];
                }
                cv::Point2d vec = cv::Point2d(result.x - neighbour.coordinates.x, result.y - neighbour.coordinates.y);
                double norm = cv::norm(vec);
                pointExtreme = (cv::Point2d)result + vec * 10 / norm;
            }
        }

        return pointExtreme;
    }

    cv::Point Graph::getSAMPointFromSkeletAllPaths(cv::Mat &skeletonImage, int blobId)
    {
        cv::Point extremity;
        cv::Point2d pointExtreme;
        std::vector<std::pair<double, cv::Point>> distances;
        double distanceMax = 0.0;
        for (auto pointNode : blobs[blobId].nodes)
        {
            if (pointNode.second.type == 1) // tip
            {
                double distance = std::min(std::min(pointNode.second.coordinates.x - 30, pointNode.second.coordinates.y), std::min(skeletonImage.cols - 30 - pointNode.second.coordinates.x, skeletonImage.rows - pointNode.second.coordinates.y));
                if (distance > distanceMax)
                {
                    distanceMax = distance;
                    extremity = pointNode.second.coordinates;
                }
            }
        }
        cv::Point2d vec;
        for (auto pointNode2 : blobs[blobId].nodes) // compute mean direction
        {
            if (pointNode2.second.coordinates == extremity)
            {
                continue;
            }
            else
            {
                vec += cv::Point2d(extremity.x - pointNode2.second.coordinates.x, extremity.y - pointNode2.second.coordinates.y);
            }
        }
        double norm = cv::norm(vec);
        pointExtreme = (cv::Point2d)extremity + vec * 10 / norm;
        return pointExtreme;
    }

    void Graph::showGraph(cv::Mat &skeletonImage)
    {
        int rapport = 2;
        cv::Mat graph;
        cv::cvtColor(skeletonImage, graph, cv::COLOR_GRAY2BGR);
        cv::resize(graph, graph, graph.size() * 2 * rapport, 0.0, 0.0, 0);
        graph = graph * 0.5;
        for (const auto &pairBlob : blobs)
        {
            cv::Point meanCoord = getMeanCoord(pairBlob.first);
            cv::circle(graph, meanCoord * 2 * rapport + cv::Point(10 * rapport, 10 * rapport), 10 * rapport, cv::Scalar(255, 0, 255), 3);
            cv::putText(graph, std::to_string(pairBlob.first), meanCoord * 2 * rapport + cv::Point(-5 * rapport, 5 * rapport) + cv::Point(10 * rapport, 10 * rapport), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);
            for (const auto &pairNodes : pairBlob.second.nodes)
            {
                if (pairNodes.second.type == 2)
                {
                    cv::rectangle(graph, cv::Point(pairNodes.second.coordinates.x * 2 * rapport, pairNodes.second.coordinates.y * 2 * rapport), cv::Point(pairNodes.second.coordinates.x * 2 * rapport + (2 * rapport - 1), pairNodes.second.coordinates.y * 2 * rapport + (2 * rapport - 1)), cv::Scalar(0, 255, 255), -1);
                }
                else
                {
                    cv::rectangle(graph, cv::Point(pairNodes.second.coordinates.x * 2 * rapport, pairNodes.second.coordinates.y * 2 * rapport), cv::Point(pairNodes.second.coordinates.x * 2 * rapport + (2 * rapport - 1), pairNodes.second.coordinates.y * 2 * rapport + (2 * rapport - 1)), cv::Scalar(0, 0, 255), -1);
                }
            }
            for (const auto &pairNodes : pairBlob.second.nodes)
            {
                cv::putText(graph, std::to_string(pairNodes.second.id), pairNodes.second.coordinates * 2 * rapport, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
                for (const auto &edge : pairNodes.second.edges)
                {
                    cv::line(graph, pairNodes.second.coordinates * 2 * rapport, pairBlob.second.nodes.at(edge.first).coordinates * 2 * rapport, cv::Scalar(0, 255, 0), 1);
                    cv::putText(graph, std::to_string((int)(edge.second * 100) / 100), (pairNodes.second.coordinates + pairBlob.second.nodes.at(edge.first).coordinates) * rapport, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                }
            }
        }
        cv::imshow("Graph", graph);
    };

    void Graph::showGraphSimple(cv::Mat &skeletonImage)
    {
        cv::Mat graph;
        cv::cvtColor(skeletonImage, graph, cv::COLOR_GRAY2BGR);
        graph = graph * 0.5;
        for (const auto &pairBlob : blobs)
        {
            for (const auto &pairNodes : pairBlob.second.nodes)
            {
                graph.at<cv::Vec3b>(pairNodes.second.coordinates) = cv::Vec3b(0, 0, 255);
            }
        }
        cv::imshow("GraphSimple", graph);
    };

}