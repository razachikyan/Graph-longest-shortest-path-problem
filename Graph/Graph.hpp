#pragma once
#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <utility>
#include <queue>
#include <algorithm>
#include <climits>
#include <limits>
#include <random>
#include <stdexcept>
#include <fstream>

#include "./Strategy/Strategy.hpp"

struct Node {
    int vertex;
    int cost;
    int heuristic;

    bool operator>(const Node& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

using NumArr = std::vector<int>;
using Matrix = std::vector<NumArr>;
using Weights = std::map<std::pair<int, int>, int>;
using Nodes = std::vector<Node>;
using Population = std::vector<NumArr>;

class Graph {
public:
    Graph() {}
    void getNeighborMatrix();
    void getGraphWeights();
    void inputGraph();
    void readGraphFromFile(const std::string& path);
    void printGraph();
    void printPath(NumArr& path);
    NumArr astar();
    NumArr dijkstra();
    NumArr genetic();
    void setStrategy(std::string strategyType);
    NumArr getShortestPath();
private:
    Weights graphWeights;
    Matrix graph;
    Population population;
    Strategy strategy;
private:
    Population initializePopulation();
    double calculateFitness(const NumArr& chromosome);
    void selection();
    void crossover();
    void mutation();
    void replacePopulation();
    void evolve();
    NumArr getBestChromosome();

private:
    int edgeCount;
    int nodeCount;
    const int INF = std::numeric_limits<int>::max();
};