#include "./Strategy.hpp"

NumArr AStarStrategy::execute(Matrix graph, int start, int end) {
    int n = graph.size();
    NumArr parent(n, -1);
    NumArr cost(n, INT_MAX);
    NumArr heuristic(n, 0);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    pq.push({ start, 0, 0 });
    cost[start] = 0;

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.vertex == end) {
            NumArr path;
            int node = end;

            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < n; ++neighbor) {
            if (graph[current.vertex][neighbor] != 0) {
                int newCost = current.cost + graph[current.vertex][neighbor];

                if (newCost < cost[neighbor]) {
                    cost[neighbor] = newCost;
                    parent[neighbor] = current.vertex;
                    pq.push({ neighbor, newCost, heuristic[neighbor] });
                }
            }
        }
    }

    return NumArr();
}

NumArr DijkstraStrategy::execute(Matrix graph, int start, int end) {
    int n = graph.size();
    const int INF = std::numeric_limits<int>::max();
    NumArr parent(n, -1);
    NumArr distance(n, INF);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    pq.push({ start, 0 });
    distance[start] = 0;

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.vertex == end) {
            NumArr path;
            int node = end;

            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < n; ++neighbor) {
            if (graph[current.vertex][neighbor] != 0) {
                int newDistance = current.cost + graph[current.vertex][neighbor];

                if (newDistance < distance[neighbor]) {
                    distance[neighbor] = newDistance;
                    parent[neighbor] = current.vertex;
                    pq.push({ neighbor, newDistance });
                }
            }
        }
    }

    return NumArr();
}