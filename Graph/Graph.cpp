#include "./Graph.hpp"

void Graph::getGraphWeights() {
    for(int i = 0; i < edgeCount; ++i) {
        std::string ans;
        std::cout << std::endl << i << " Enter node1 node2 and weight in this format [node1] [node2] [weight]";
        std::getline(std::cin, ans);
        std::istringstream iss(ans);
        std::string word;
        int ind = 0;
        int weight;
        std::pair<int, int> neighborNodes = {0, 0};
        while (iss >> word) {
            switch(ind) {
                case 0: {
                    neighborNodes.first = std::stoi(word);
                    break;
                }
                case 1: {
                    neighborNodes.second = std::stoi(word);
                    break;
                }
                case 2: {
                    weight = std::stoi(word);
                    break;
                }
                default: throw std::runtime_error("Too meny input values. Not valid!.");
            }
            ++ind;
        }
        graphWeights[neighborNodes] = weight;
    }
};

void Graph::getNeighborMatrix() {
    for(int i = 0; i < nodeCount; ++ i) {
        for(int j = 0; j < nodeCount; ++ j) {
            auto weight = graphWeights.find(std::make_pair(i, j));
            neighborMatrix[i][j] = (weight != graphWeights.end()) ? weight->second : 0;
        }
    }
};

NumArr Graph::astar() {
    int start, target;
    std::cout << "enter start and target nodes"; std::cin>>start>>target;
    int n = neighborMatrix.size();
    NumArr parent(n, -1);
    NumArr cost(n, INT_MAX);
    NumArr heuristic(n, 0);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    pq.push({start, 0, 0});
    cost[start] = 0;

        while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.vertex == target) {
            NumArr path;
            int node = target;

            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < n; ++neighbor) {
            if (neighborMatrix[current.vertex][neighbor] != 0) {
                int newCost = current.cost + neighborMatrix[current.vertex][neighbor];

                if (newCost < cost[neighbor]) {
                    cost[neighbor] = newCost;
                    parent[neighbor] = current.vertex;
                    pq.push({neighbor, newCost, heuristic[neighbor]});
                }
            }
        }
    }

    return NumArr();
}

void Graph::printMatrix()
{
    std::cout << "  ";
    for( int i = 0; i < neighborMatrix.size(); ++i )
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    for( int i = 0; i < neighborMatrix.size(); ++i )
    {
        std::cout << i << " ";
        for( int j = 0; j < neighborMatrix.size(); ++j )
        {
            std::cout << neighborMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void Graph::inputGraph()
{
    std::cout << "Input Node and Edge count: "; std::cin >> nodeCount >> edgeCount;

    neighborMatrix = std::vector< std::vector< int > >( nodeCount, std::vector< int >( nodeCount, 0 ) );

    for ( int i = 0; i < edgeCount; ++i )
    {
        int node1, node2, weight;
        std::cout << "Input node_1, node_2 and weight: "; std::cin >> node1 >> node2 >> weight;
        neighborMatrix[node1][node2] = weight;
        neighborMatrix[node2][node1] = weight;
    }
}