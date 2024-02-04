#pragma once

#include <queue>
#include <vector>
#include <limits.h>
#include <limits>
#include <algorithm>

using NumArr = std::vector<int>;
using Matrix = std::vector<NumArr>;

struct Node {
    int vertex;
    int cost;
    int heuristic;

    bool operator>(const Node& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

class Strategy {
public:
    virtual NumArr execute(Matrix graph, int start, int end) const = 0;
    virtual ~Strategy() = default;
};

class AStarStrategy:public Strategy {
public:
    NumArr execute(Matrix graph, int start, int end) const override;
};

class DijkstraStrategy:public Strategy {
public:
    NumArr execute(Matrix graph, int start, int end)  const override;
};