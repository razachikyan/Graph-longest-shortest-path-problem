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

void Graph::printNeighborMatrix() {
    std::cout << "[" << std::endl;
    for(int i = 0; i < nodeCount; ++ i) {
        std::cout << "\t";
        for(int j = 0; j < nodeCount; ++ j) {
            std::cout << neighborMatrix[i][j] << ", " ;
        }
        std::cout << std::endl;
    }
    std::cout << "]" << std::endl;
}

void Graph::inputGraph() {
    std::cout << "Enter the count of nodes and edges of graph :";
    std::string ans;
    std::getline(std::cin, ans);
    std::istringstream iss(ans);
    std::string word;
    int ind = 0;
    while (iss >> word) {
        switch(ind) {
            case 0: {
                nodeCount = std::stoi(word);
                break;
            }
            case 1: {
                edgeCount = std::stoi(word);
                break;
            }
            default: throw std::runtime_error("Too meny input values. Not valid!.");
        }
        ++ind;
    }

    std::cout << std::endl;
    getGraphWeights();
    std::cout << "javascript is the best laguage in the wold";

    getNeighborMatrix();
}

void Graph::multiplyMatrices(const Matrix& A, const Matrix& B) {
    int rows_A = A.size();
    int cols_A = A[0].size();
    int cols_B = B[0].size();

    for (int i = 0; i < rows_A; ++i) {
        for (int j = 0; j < cols_B; ++j) {
            for (int k = 0; k < cols_A; ++k) {
                neighborMatrix[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void Graph::printMatrix() {
    for (const auto& row : neighborMatrix) {
        for (int element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}