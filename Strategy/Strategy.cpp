#include "./Strategy.hpp"

NumArr AStarStrategy::execute(Matrix adjacencyMatrix, int source, int target) const {
    int nodeCount = adjacencyMatrix.size();
    NumArr parent(nodeCount, -1);
    NumArr cost(nodeCount, INT_MAX);
    NumArr heuristic(nodeCount, 0);

    std::priority_queue<NodeAstar, std::vector<NodeAstar>, std::greater<NodeAstar>> vertexPriorityQueue;

    vertexPriorityQueue.push({ source, 0, 0 });
    cost[source] = 0;

    while (!vertexPriorityQueue.empty()) {
        NodeAstar currentNode = vertexPriorityQueue.top();
        vertexPriorityQueue.pop();

        if (currentNode.name == target) {
            NumArr shortestPath;
            int node = target;

            while (node != -1) {
                shortestPath.push_back(node);
                node = parent[node];
            }

            std::reverse(shortestPath.begin(), shortestPath.end());
            return shortestPath;
        }

        for (int neighbor = 0; neighbor < nodeCount; ++neighbor) {
            int edgeWeight = adjacencyMatrix[currentNode.name][neighbor];
            if (edgeWeight != 0) {
                int newCost = currentNode.cost + edgeWeight;

                if (newCost < cost[neighbor]) {
                    cost[neighbor] = newCost;
                    parent[neighbor] = currentNode.name;
                    int newHeuristic = heuristic[neighbor];  // Replace with the actual heuristic calculation
                    vertexPriorityQueue.push({ neighbor, newCost, newHeuristic });
                }
            }
        }
    }

    return NumArr();
}

// NumArr DijkstraStrategy::execute(Matrix adjacencyMatrix, int source, int target) const {
//     int nodeCount = adjacencyMatrix.size();
//     const int INF = std::numeric_limits<int>::max();
//     NumArr parent(nodeCount, -1);
//     NumArr distance(nodeCount, INF);

//     std::vector<NodeDijkstra> nodesPriorityQueue;
//     std::cout << source << std::endl;
//     nodesPriorityQueue.push_back({ source, 0 });
//     distance[source] = 0;

//     while (!nodesPriorityQueue.empty()) {
//         NodeDijkstra currentNode = nodesPriorityQueue[nodesPriorityQueue.size() - 1];
//         nodesPriorityQueue.pop_back();

//         if (currentNode.name == target) {
//             NumArr shortestPath;
//             int node = target;

//             while (node != -1) {
//                 shortestPath.push_back(node);
//                 node = parent[node];
//             }

//             std::reverse(shortestPath.begin(), shortestPath.end());
//             return shortestPath;
//         }

//         for (int neighbor = 0; neighbor < nodeCount; ++neighbor) {
//             int edgeWeight = adjacencyMatrix[currentNode.name][neighbor];
//             if (edgeWeight != 0) {
//                 int newDistance = currentNode.cost + edgeWeight;
//                 if (newDistance < distance[neighbor]) {
//                     std::cout << "currentNode.cost::" << currentNode.cost << " + edgeWeight::" << edgeWeight << " = " << newDistance << std::endl;
//                     distance[neighbor] = newDistance;
//                     parent[neighbor] = currentNode.name;
//                     nodesPriorityQueue.push_back({ neighbor, newDistance });
//                 }
//             }
//         }
//     }

//     return NumArr();
// }


NumArr DijkstraStrategy::execute(Matrix adjacencyMatrix, int source, int target) const {
    int nodeCount = adjacencyMatrix.size();
    NumArr dist(nodeCount);
    NumArr Tset(nodeCount);

    for (int i = 0; i < nodeCount; ++i) {
        dist[i] = 1000;
        Tset[i] = 0;
    }

    dist[source] = 0;

    for (int node = 0; node < nodeCount; node++) {
        int min = INT_MAX, currentNode = 0;
        for (int node = 0; node < nodeCount; node++) {
            if (Tset[node] == 0 && dist[node] <= min) {
                min = dist[node];
                currentNode = node;
            }
        }

        Tset[currentNode] = 1;

        for (int node = 0; node < nodeCount; node++) {
            if (Tset[node] == 0 && adjacencyMatrix[currentNode][node] && dist[currentNode] != INT_MAX && dist[currentNode] + adjacencyMatrix[currentNode][node] < dist[node]) {
                dist[node] = dist[currentNode] + adjacencyMatrix[currentNode][node];
            }
        }
    }
    return dist;
}