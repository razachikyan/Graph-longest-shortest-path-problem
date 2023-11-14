#pragma once
#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <utility>
#include <queue>
#include <algorithm>
#include <climits>

using NumArr = std::vector<int>;
using Matrix = std::vector<NumArr>;
using Weights = std::map<std::pair<int, int>, int>;
struct Node {
    int vertex;
    int cost;
    int heuristic;

    bool operator>(const Node& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};
class Graph {
public:
    Graph() {}
    void getNeighborMatrix();
    void getGraphWeights();
    void inputGraph();
    void printMatrix();
    NumArr astar(const Matrix& graph, int start, int goal);
private:
    Weights graphWeights;
    Matrix neighborMatrix;
    int edgeCount;
    int nodeCount;
};