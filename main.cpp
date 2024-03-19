#include "./Models/Graph/Graph.hpp"

int main() {
    Graph graph;
    graph.readGraphFromFile("graph.txt");
    graph.printGraph();
    NumArr path = graph.GAIP(10);

    graph.printPath(path);

    return 0;
}