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
#include <limits>

struct Node {
    int vertex;
    int cost;
    int heuristic;

    bool operator>(const Node& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};
using NumArr = std::vector<int>;
using Matrix = std::vector<NumArr>;
using Weights = std::map<std::pair<int, int>, int>;
using Nodes = std::vector<Node>;
class Graph {
public:
    Graph() {}
    void getNeighborMatrix();
    void getGraphWeights();
    void inputGraph();
    void printMatrix();
    NumArr astar();
    NumArr dijkstra();
private:
    Weights graphWeights;
    Matrix neighborMatrix;
    int edgeCount;
    int nodeCount;
    const int INF = std::numeric_limits<int>::max();
};