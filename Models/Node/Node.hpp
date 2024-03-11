#pragma once

#include "../../libs.hpp"

class Node {
public:
    Node(int index, std::string name): index(index), name(name) {};
    std::string getName();
    int getIndex();
private:
    int index;
    std::string name;
};