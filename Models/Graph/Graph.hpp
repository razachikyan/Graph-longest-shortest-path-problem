#pragma once

#include "../../Strategy/Strategy.hpp"
#include "../Node/Node.hpp"

using Nodes = std::vector<Node>;
using Population = std::vector<NumArr>;
using Weights = std::map<std::pair<int, int>, int>;

class Graph {
public:
    Graph(): strategy(nullptr) {}
    NumArr genetic();
    void inputGraph();
    void printGraph();
    void setNodes();
    void getGraphWeights();
    void writeGraphToFile();
    void getNeighborMatrix();
    NumArr getShortestPath();
    void printPath(NumArr& path);
    void setStrategy(std::string strategyType);
    void readGraphFromFile(const std::string& path);
private:
    Matrix graph;
    Weights graphWeights;
    Population population;
    std::unique_ptr<Strategy> strategy;

private:
    void evolve();
    void mutation();
    void selection();
    void crossover();
    void replacePopulation();
    NumArr getBestChromosome();
    Population initializePopulation();
    double calculateFitness(const NumArr& chromosome);
    int findNodeByName(std::string);
private:
    int edgeCount;
    int nodeCount;
    std::vector<Node> nodes;
    const int INF = std::numeric_limits<int>::max();
};