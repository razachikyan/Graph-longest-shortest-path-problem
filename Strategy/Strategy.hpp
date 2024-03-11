#pragma once

#include "../libs.hpp"

using NumArr = std::vector<int>;
using Matrix = std::vector<NumArr>;

struct NodeStr {
    int cost;
    int vertex;
    int heuristic;

    bool operator>(const NodeStr& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

class Strategy {
public:
    virtual ~Strategy() = default;
    virtual NumArr execute(Matrix graph, int start, int end) const = 0;
};

class AStarStrategy:public Strategy {
public:
    NumArr execute(Matrix graph, int start, int end) const override;
};

class DijkstraStrategy:public Strategy {
public:
    NumArr execute(Matrix graph, int start, int end)  const override;
};