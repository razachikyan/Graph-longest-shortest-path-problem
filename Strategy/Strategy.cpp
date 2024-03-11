#include "./Strategy.hpp"

NumArr AStarStrategy::execute(Matrix adjacencyMatrix, int startNode, int targetNode) const {
    int nodeCount = adjacencyMatrix.size();
    NumArr parent(nodeCount, -1);
    NumArr cost(nodeCount, INT_MAX);
    NumArr heuristic(nodeCount, 0);

    std::priority_queue<NodeStr, std::vector<NodeStr>, std::greater<NodeStr>> vertexPriorityQueue;

    vertexPriorityQueue.push({ startNode, 0, 0 });
    cost[startNode] = 0;

    while (!vertexPriorityQueue.empty()) {
        NodeStr currentVertex = vertexPriorityQueue.top();
        vertexPriorityQueue.pop();

        if (currentVertex.vertex == targetNode) {
            NumArr shortestPath;
            int node = targetNode;

            while (node != -1) {
                shortestPath.push_back(node);
                node = parent[node];
            }

            std::reverse(shortestPath.begin(), shortestPath.end());
            return shortestPath;
        }

        for (int neighbor = 0; neighbor < nodeCount; ++neighbor) {
            int edgeWeight = adjacencyMatrix[currentVertex.vertex][neighbor];
            if (edgeWeight != 0) {
                int newCost = currentVertex.cost + edgeWeight;

                if (newCost < cost[neighbor]) {
                    cost[neighbor] = newCost;
                    parent[neighbor] = currentVertex.vertex;
                    int newHeuristic = heuristic[neighbor];  // Replace with the actual heuristic calculation
                    vertexPriorityQueue.push({ neighbor, newCost, newHeuristic });
                }
            }
        }
    }

    return NumArr();
}

NumArr DijkstraStrategy::execute(Matrix adjacencyMatrix, int startNode, int targetNode) const {
    int nodeCount = adjacencyMatrix.size();
    const int INF = std::numeric_limits<int>::max();
    NumArr parent(nodeCount, -1);
    NumArr distance(nodeCount, INF);

    std::priority_queue<NodeStr, std::vector<NodeStr>, std::greater<NodeStr>> nodesPriorityQueue;

    nodesPriorityQueue.push({ startNode, 0 });
    distance[startNode] = 0;

    while (!nodesPriorityQueue.empty()) {
        NodeStr currentVertex = nodesPriorityQueue.top();
        nodesPriorityQueue.pop();

        if (currentVertex.vertex == targetNode) {
            NumArr shortestPath;
            int node = targetNode;

            while (node != -1) {
                shortestPath.push_back(node);
                node = parent[node];
            }

            std::reverse(shortestPath.begin(), shortestPath.end());
            return shortestPath;
        }

        for (int neighbor = 0; neighbor < nodeCount; ++neighbor) {
            int edgeWeight = adjacencyMatrix[currentVertex.vertex][neighbor];
            if (edgeWeight != 0) {
                int newDistance = currentVertex.cost + edgeWeight;

                if (newDistance < distance[neighbor]) {
                    distance[neighbor] = newDistance;
                    parent[neighbor] = currentVertex.vertex;
                    nodesPriorityQueue.push({ neighbor, newDistance });
                }
            }
        }
    }

    return NumArr();
}
