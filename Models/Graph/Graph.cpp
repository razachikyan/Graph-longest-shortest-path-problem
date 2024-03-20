#include "./Graph.hpp"

void Graph::getGraphWeights() {
    for (int i = 0; i < edgeCount; ++i) {
        std::string first;
        std::string second;
        int weight;
        std::cout << "Input node_1, node_2 and weight: "; std::cin >> first >> second >> weight;
        int node1 = findNodeByName(first);
        int node2 = findNodeByName(second);
        graphWeights[{node1, node2}] = weight;
    }
};

void Graph::getNeighborMatrix() {
    graph = Matrix(nodeCount, NumArr(nodeCount, 0));
    for (auto& pair : graphWeights) {
        int node1 = pair.first.first, node2 = pair.first.second, weight = pair.second;
        graph[node1][node2] = weight;
        graph[node2][node1] = weight;
    }
};

void Graph::printGraph() {
    std::cout << std::left << std::setfill(' ') << std::setw(4);
    for (int i = 0; i < graph.size(); ++i) std::cout << "*" << std::left << std::setfill(' ') << std::setw(4);
    std::cout << std::endl;
    std::cout << std::endl;

    for (int i = 0; i < graph.size(); ++i) {
        std::cout << "*" << std::left << std::setfill(' ') << std::setw(4);
        for (int j = 0; j < graph.size(); ++j) std::cout << graph[i][j] << std::left << std::setfill(' ') << std::setw(4);
        std::cout << std::endl << std::endl;
    }
}

void Graph::printPath(NumArr& path) {
    for (int i = 0; i < path.size(); ++i) {
        std::cout << nodes[path[i]].getName() << " ";
    }
    std::cout << std::endl;
}

void Graph::inputGraph() {
    std::cout << "Input Node and Edge count: "; std::cin >> nodeCount >> edgeCount;

    setNodes();
    getGraphWeights();
    getNeighborMatrix();
}

void Graph::setNodes() {
    for (int i = 0; i < nodeCount; ++i) {
        std::string name;
        std::cout << "Enter the node name:: "; std::cin >> name;
        Node node = Node(i, name);
        nodes.push_back(node);
    }
}

void Graph::writeGraphToFile() {
    const std::string outputFileName = "graphOutput.txt";
    std::ofstream outputFile(outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
        return;
    }

    for (const auto& entry : graphWeights) {
        std::string node1 = nodes[entry.first.first].getName();
        std::string node2 = nodes[entry.first.second].getName();
        outputFile << node1 << ' ' << node1 << ' ' << entry.second << '\n';
    }

    outputFile.close();
    std::cout << "Data has been written to " << outputFileName << std::endl;
}

void Graph::readGraphFromFile(const std::string& path) {
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    std::string node1, node2;
    int weight;
    int id = 0;
    std::vector<std::string> nodeVec;

    while (inputFile >> node1 >> node2 >> weight) {
        int first = findNodeByName(node1);
        int second = findNodeByName(node2);

        if (first == -1) {
            Node node = Node(nodes.size(), node1);
            nodes.push_back(node);
            first = node.getIndex();
        }

        if (second == -1) {
            Node node = Node(nodes.size(), node2);
            nodes.push_back(node);
            second = node.getIndex();
        }

        graphWeights[{first, second}] = weight;
    }

    inputFile.close();

    nodeCount = nodes.size();
    edgeCount = graphWeights.size();

    getNeighborMatrix();
};

double Graph::calculateFitness(const NumArr& chromosome) {
    const int maxPathLength = std::numeric_limits<int>::max();
    int pathLength = 0;
    for (size_t i = 0; i < chromosome.size() - 1; ++i) {
        int currentVertex = chromosome[i];
        int nextVertex = chromosome[i + 1];
        if (currentVertex >= graph.size() || nextVertex >= graph.size()) {
            pathLength += maxPathLength;
        }
        else {
            if (graph[currentVertex][nextVertex] < maxPathLength - pathLength) {
                pathLength += graph[currentVertex][nextVertex];
            }
            else {
                pathLength = maxPathLength;
                break;
            }
        }
    }
    return (pathLength == 0) ? std::numeric_limits<double>::infinity() : 1.0 / static_cast<double>(pathLength + 1);
}

Individual Graph::generateRandomIndividual() {
    NumArr path(nodeCount);
    for (int i = 0; i < nodeCount; ++i)
        path[i] = i;

    std::shuffle(path.begin(), path.end(), std::mt19937(std::random_device()()));

    return Individual(path);
}

double Graph::evaluateFitness(const Individual& ind) {
    double length = 0.0;
    for (int i = 1; i < nodeCount; ++i)
        length += graph[ind.path[i - 1]][ind.path[i]];

    return length;
}

NumArr Graph::GAIP(int generations) {
    std::vector<Individual> population;
    for (int i = 0; i < 100; ++i) {
        population.push_back(generateRandomIndividual());
    }

    Individual bestIndividual = population[0];
    for (int gen = 0; gen < generations; ++gen) {
        for (auto& ind : population) {
            ind.fitness = evaluateFitness(ind);
        }

        std::sort(population.begin(), population.end(), [](const Individual& a, const Individual& b){
            return a.fitness < b.fitness;
        });

        if (population[0].fitness < bestIndividual.fitness) {
            bestIndividual = population[0];
        }
    }

    return bestIndividual.path;
}

void Graph::setStrategy(std::string strategyType) {
    if (strategyType == "dijkstra") this->strategy = std::make_unique<DijkstraStrategy>();
    else if (strategyType == "astar") this->strategy = std::make_unique<AStarStrategy>();
    else throw std::runtime_error("Incorrect strategy type");
}

NumArr Graph::getShortestPath() {
    if (!this->strategy) throw std::runtime_error("Choose strategy for getting shortest path");
    std::string start, target;
    std::cout << "enter start and target nodes [a b]:: "; std::cin >> start >> target;
    int node1 = findNodeByName(start);
    int node2 = findNodeByName(target);
    if (node1 != -1 && node2 != -1) return strategy->execute(graph, node1, node2);
    return {};
}

int Graph::findNodeByName(std::string name) {
    for (int i = 0; i < nodes.size(); ++i) {
        if (nodes[i].getName() == name) return i;
    }
    return -1;
}