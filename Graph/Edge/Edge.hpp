#pragma once
#include "../Node/Node.hpp"

class Edge {
public:
    Edge(Node &nodeA, Node &nodeB, int weight): nodeA{nodeA}, nodeB{nodeB}, weight{weight} {};
private:
    Node nodeA, nodeB;
    int weight;
};