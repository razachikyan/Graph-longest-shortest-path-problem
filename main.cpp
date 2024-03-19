#include "./Models/Graph/Graph.hpp"

int main() {
    Graph graph;
    graph.readGraphFromFile("graph.txt");
    graph.printGraph();
    graph.setStrategy("dijkstra");
    NumArr pathDijkstra = graph.getShortestPath(); std::cout << std::endl;
    graph.printPath(pathDijkstra);
    std::cout << pathDijkstra.size() << std::endl;

    graph.setStrategy("astar");
    NumArr pathAStar = graph.getShortestPath();
    std::cout << "Shortest path >>>> " << pathAStar.size() << std::endl;
    graph.printPath(pathAStar);

    return 0;
}