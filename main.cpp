#include "./Graph/Graph.hpp"

int main() {
    Graph graph;
    graph.inputGraph();
    graph.printMatrix();
    std::vector<int> path = graph.astar();
    for(const auto& node: path) {
        std::cout << node << ", ";
    }
    return 0;
}