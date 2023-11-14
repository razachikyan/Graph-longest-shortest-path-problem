#pragma once
#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <utility>

using numArr = std::vector<int>;
using Matrix = std::vector<numArr>;
using Weights = std::map<std::pair<int, int>, int>;

class Graph {
public:
    Graph() {}
    void printNeighborMatrix();
    void getNeighborMatrix();
    void getGraphWeights();
    void inputGraph();
    void printMatrix();
    void multiplyMatrices(const Matrix& A, const Matrix& B);
private:
    Weights graphWeights;
    Matrix neighborMatrix;
    int edgeCount;
    int nodeCount;
};


// Function to print a matrix
void printMatrix(const Matrix& matrix) {
    for (const auto& row : matrix) {
        for (int element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}