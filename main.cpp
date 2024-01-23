#include "./Graph/Graph.hpp"

int main() {
    Graph graph;
    graph.readGraphFromFile("graph.txt");

    std::cout << "\tShortest path with dijkstra\n";

    NumArr pathDijkstra = graph.dijkstra();
    graph.printPath(pathDijkstra);

    std::cout << "\tShortest path with A*\n";
    NumArr pathAStar = graph.astar();
    graph.printPath(pathAStar);

    return 0;
}