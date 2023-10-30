#pragma once
#include <vector>
#include <iostream>
#include "./Edge/Edge.hpp"
#include "./Node/Node.hpp"

using Edges = std::vector<Edge>;
using Nodes = std::vector<Node>;


class Graph {
public:
    Graph(int edgeCount, int nodeCount): edgeCount{edgeCount}, nodeCount{nodeCount} {};
    void inputGraphNodes();
    void inputGraphEdges();
    void inputGraph();
private:
    int edgeCount;
    int nodeCount;
    Edges edges;
    Nodes nodes;
};