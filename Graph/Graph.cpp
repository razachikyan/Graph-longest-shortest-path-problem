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

Matrix Graph::cloneMatrix(const Matrix &matrix) {
    Matrix res;
    for (const auto& row : matrix) {
        res.push_back(row);
        for (int element : row) {
            res[res.size() -1].push_back(element);
        }
    }

    return res;
}

Matrix Graph::getMatrixPow(int pow,const Matrix &matrix) {
    Matrix res = cloneMatrix(neighborMatrix);
    if(pow <= 2) {
        return multiplyMatrices(res, neighborMatrix);
    } else {
        Matrix temp = multiplyMatrices(res, matrix);
        return getMatrixPow(pow-1, temp);
    }
}

Matrix Graph::multiplyMatrices(const Matrix &matrix1, const Matrix &matrix2) {
    Matrix res = cloneMatrix(neighborMatrix);
    int rows_A = matrix1.size();
    int cols_A = matrix1[0].size();
    int cols_B = matrix2[0].size();
    for (int i = 0; i < rows_A; ++i) {
        for (int j = 0; j < cols_B; ++j) {
            for (int k = 0; k < cols_A; ++k) {
                res[i][j] += res[i][k] * neighborMatrix[k][j];
            }
        }
    }
    return res;
}

void Graph::printMatrix() {
    for (const auto& row : neighborMatrix) {
        for (int element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}