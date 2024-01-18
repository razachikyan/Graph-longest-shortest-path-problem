#include "./Graph/Graph.hpp"

int main() {
    Graph graph;
    graph.inputGraph();

    std::cout << "Your matrix >>\n";

    graph.printMatrix();

    std::cout << "Print the longest path(L).\nPrint the shortest path between 2 nodes(S)\n";
    std::string ans;
    std::cin >> ans;
    std::cout << std::endl;

    if (ans == "L") {
        std::vector<int> longPath = graph.getLongestPath();

        for (const auto& node : longPath) {
            std::cout << node << ", ";
        }
    }
    else if (ans == "S") {
        std::vector<int> path = graph.astar();

        for (const auto& node : path) {
            std::cout << node << ", ";
        }
    }
    else {
        std::cout << "Wrong answer\n";
    }


    return 0;
}