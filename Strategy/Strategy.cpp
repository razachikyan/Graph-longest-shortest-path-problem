#include "./Strategy.hpp"

NumArr AStarStrategy::execute(Matrix adjacencyMatrix, int source, int target) const {
    int nodeCount = adjacencyMatrix.size();
    NumArr parent(nodeCount, -1);
    NumArr cost(nodeCount, INT_MAX);
    NumArr heuristic(nodeCount, 0);

    std::priority_queue<NodeAstar, std::vector<NodeAstar>, std::greater<NodeAstar>> nodeQueue;

    nodeQueue.push({ source, 0, 0 });
    cost[source] = 0;
    while (!nodeQueue.empty()) {
        NodeAstar current = nodeQueue.top();
        nodeQueue.pop();

        if (current.name == target) {
            NumArr path;
            int node = target;

            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < nodeCount; ++neighbor) {
            if (adjacencyMatrix[current.name][neighbor] != 0) {
                int newCost = current.cost + adjacencyMatrix[current.name][neighbor];

                if (newCost < cost[neighbor]) {
                    cost[neighbor] = newCost;
                    parent[neighbor] = current.name;
                    nodeQueue.push({ neighbor, newCost, heuristic[neighbor] });
                }
            }
        }
    }

    return NumArr();
}

NumArr DijkstraStrategy::execute(Matrix adjacencyMatrix, int source, int target) const {
    int nodeCount = adjacencyMatrix.size();
    NumArr parent(nodeCount, -1);
    NumArr distance(nodeCount, INT_MAX);

    std::priority_queue<NodeDijkstra, std::vector<NodeDijkstra>, std::greater<NodeDijkstra>> nodeQueue;

    nodeQueue.push({ source, 0 });
    distance[source] = 0;

    while (!nodeQueue.empty()) {
        NodeDijkstra current = nodeQueue.top();
        nodeQueue.pop();

        if (current.name == target) {
            NumArr path;
            int node = target;

            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < nodeCount; ++neighbor) {
            if (adjacencyMatrix[current.name][neighbor] != 0) {
                int newDistance = current.cost + adjacencyMatrix[current.name][neighbor];

                if (newDistance < distance[neighbor]) {
                    distance[neighbor] = newDistance;
                    parent[neighbor] = current.name;
                    nodeQueue.push({ neighbor, newDistance });
                }
            }
        }
    }

    return NumArr();
}

void bruteForce(Matrix& adjMatrix, int current, int end, NumArr& currentPath, int pathLength, int& maxPathLength, NumArr& maxPath) {
    if (current == end) {
        if (pathLength > maxPathLength) {
            maxPathLength = pathLength;
            maxPath = currentPath;
        }
        return;
    }

    for (int i = 0; i < adjMatrix.size(); i++) {
        if (adjMatrix[current][i] != 0 && find(currentPath.begin(), currentPath.end(), i) == currentPath.end()) {
            currentPath.push_back(i);
            bruteForce(adjMatrix, i, end, currentPath, pathLength + adjMatrix[current][i], maxPathLength, maxPath);
            currentPath.pop_back();
        }
    }
}

NumArr BruteForceStrategy::execute(Matrix adjMatrix, int start, int end) const {
    int maxPathLength = INT_MIN;
    NumArr maxPath;

    NumArr currentPath;
    currentPath.push_back(start);

    bruteForce(adjMatrix, start, end, currentPath, 0, maxPathLength, maxPath);

    return maxPath;
}
