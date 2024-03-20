#include "./Models/Graph/Graph.hpp"

int main() {
    Graph graph;
    graph.readGraphFromFile("graph.txt");
    // graph.printGraph();
    NumArr path = graph.GAIP(10);
    // graph.printPath(path);
    std::cout << std::endl;
    for(auto i : path) {
        std::cout << path[i] << ", ";
    }
    std::cout << std::endl;
    return 0;
}