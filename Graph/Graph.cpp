#include "./Graph.hpp"

void Graph::inputGraphNodes() {
    for(int i = 0; i < nodeCount; ++i) {
        int node;
        std::cout << std::endl << "Enter node: ";
        std::cin >> node;
        nodes.push_back(Node(node));
    };
}

void Graph::inputGraphEdges() {
    for(int i = 0; i < edgeCount; ++i) {
        int weight;
        int node1;
        int node2;
        std::cout << std::endl << "Enter node1: ";
        std::cin >> node1;
        std::cout << std::endl << "Enter node2: ";
        std::cin >> node1;
        std::cout << std::endl << "Enter weight: ";
        std::cin >> weight;
        edges.push_back(Edge(Node(node1), Node(node2), weight));
    };
}

void Graph::inputGraph() {
    inputGraphNodes();
    inputGraphEdges();
}