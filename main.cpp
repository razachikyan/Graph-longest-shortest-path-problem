#include "./Graph/Graph.hpp"

int main() {
    Graph graph;
    graph.inputGraph();

    std::cout << "Your matrix >>\n";

    graph.printMatrix();
    std::cout << std::endl << std::endl << std::endl;

    NumArr path = graph.getLongestPath();

    for(int i = 0; i < path.size(); ++i) {
        std::cout << path[i] << ", ";
    }
    std::cout << std::endl;

    return 0;
}