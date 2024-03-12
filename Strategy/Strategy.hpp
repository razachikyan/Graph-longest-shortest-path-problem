#pragma once

#include "../libs.hpp"

using NumArr = std::vector<int>;
using Matrix = std::vector<NumArr>;

struct NodeAstar {
    int cost;
    int name;
    int heuristic;

    bool operator>(const NodeAstar& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

struct NodeDijkstra {
    int name;
    int cost;
};

class Strategy {
public:
    virtual ~Strategy() = default;
    virtual NumArr execute(Matrix graph, int start, int end) const = 0;
};

class DijkstraStrategy :public Strategy {
public:
    NumArr execute(Matrix graph, int start, int end)  const override;
};


class AStarStrategy :public Strategy {
public:
    NumArr execute(Matrix graph, int start, int end) const override;
};