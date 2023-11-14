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
    Matrix multiplyMatrices(const Matrix &matrix,const Matrix &matrix2);
    Matrix getMatrixPow(int pow, const Matrix &matrix);
    Matrix cloneMatrix(const Matrix &matrix1);
private:
    Weights graphWeights;
    Matrix neighborMatrix;
    int edgeCount;
    int nodeCount;
};