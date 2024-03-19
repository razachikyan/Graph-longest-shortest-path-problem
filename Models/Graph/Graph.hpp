#pragma once

#include "../../Strategy/Strategy.hpp"
#include "../Node/Node.hpp"

using Nodes = std::vector<Node>;
using Population = std::vector<NumArr>;
using Weights = std::map<std::pair<int, int>, int>;

struct Individual {
    NumArr path;
    double fitness;

    Individual(const NumArr& p) : path(p), fitness(0.0) {}
};

class Graph {
public:
    Graph() : strategy(nullptr) {}
    void setNodes();
    NumArr genetic();
    void inputGraph();
    void printGraph();
    void getGraphWeights();
    void writeGraphToFile();
    void getNeighborMatrix();
    NumArr getShortestPath();
    void printPath(NumArr& path);
    NumArr GAIP(int generations);
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
    int findNodeByName(std::string);
    Population initializePopulation();
    Individual generateRandomIndividual();
    double evaluateFitness(const Individual& ind);
    double calculateFitness(const NumArr& chromosome);
    bool compareIndividuals(const Individual& a, const Individual& b);
private:
    int edgeCount;
    int nodeCount;
    std::vector<Node> nodes;
    const int INF = std::numeric_limits<int>::max();
};