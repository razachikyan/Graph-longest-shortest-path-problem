#include "./Graph/Graph.hpp"

int main() {
    Graph graph;
    graph.inputGraph();
    NumArr path = graph.getLongestPath();

    for (int i = 0; i < path.size(); ++i) {
        std::cout << path[i] << ", ";
    }
    std::cout << std::endl;

    return 0;
}