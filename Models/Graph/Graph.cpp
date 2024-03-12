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
    graph = std::vector< std::vector<int>>(nodeCount, std::vector<int>(nodeCount, 0));
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

Population Graph::initializePopulation() {
    population.clear();
    const int populationSize = 50;

    for (int i = 0; i < populationSize; ++i) {
        NumArr chromosome;
        for (int vertex = 0; vertex < graph.size(); ++vertex) {
            std::cout << graph.size() << "/" << vertex << std::endl;
            chromosome.push_back(vertex);
        }
        std::shuffle(chromosome.begin(), chromosome.end(), std::default_random_engine(std::random_device()()));
        population.push_back(chromosome);
    }
}

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

void Graph::selection() {
    const int populationSize = 50;
    double totalFitness = 0.0;
    std::vector<double> fitnessValues;
    fitnessValues.reserve(population.size());

    for (const auto& chromosome : population) {
        double fitness = calculateFitness(chromosome);
        totalFitness += fitness;
        fitnessValues.push_back(fitness);
    }

    std::vector<double> cumulativeProbabilities;
    cumulativeProbabilities.reserve(population.size());
    double cumulativeProbability = 0.0;

    for (double fitness : fitnessValues) {
        double probability = fitness / totalFitness;
        cumulativeProbability += probability;
        cumulativeProbabilities.push_back(cumulativeProbability);
    }

    Population selectedChromosomes;

    for (int i = 0; i < populationSize; ++i) {
        double randomValue = static_cast<double>(rand()) / RAND_MAX;
        auto selectedIt = std::upper_bound(cumulativeProbabilities.begin(), cumulativeProbabilities.end(), randomValue);
        int selectedIndex = static_cast<int>(std::distance(cumulativeProbabilities.begin(), selectedIt));
        selectedChromosomes.push_back(population[selectedIndex]);
    }

    population = selectedChromosomes;
}


void Graph::crossover() {
    Population newPopulation;
    const size_t crossoverPointRange = 1;

    for (size_t i = 0; i < population.size(); i += 2) {
        if (i + 1 < population.size()) {
            const NumArr& parent1 = population[i];
            const NumArr& parent2 = population[i + 1];
            size_t crossoverPoint = rand() % (parent1.size() - crossoverPointRange) + crossoverPointRange;
            NumArr offspring1(parent1.begin(), parent1.begin() + crossoverPoint);
            offspring1.insert(offspring1.end(), parent2.begin() + crossoverPoint, parent2.end());
            NumArr offspring2(parent2.begin(), parent2.begin() + crossoverPoint);
            offspring2.insert(offspring2.end(), parent1.begin() + crossoverPoint, parent1.end());
            newPopulation.push_back(offspring1);
            newPopulation.push_back(offspring2);
        }

        else newPopulation.push_back(population[i]);
    }

    population = newPopulation;
}


void Graph::mutation() {
    const double mutationRate = 0.1;
    auto getRandomPosition = [&]() -> size_t {
        return rand() % population.front().size();
        };

    for (auto& chromosome : population) {
        if ((static_cast<double>(rand()) / RAND_MAX) < mutationRate) {
            size_t position1 = getRandomPosition();
            size_t position2;
            do {
                position2 = getRandomPosition();
            } while (position1 == position2);

            std::swap(chromosome[position1], chromosome[position2]);
        }
    }
}


void Graph::replacePopulation() {
    const size_t elitismCount = 2;
    Population combinedPopulation = population;
    std::sort(combinedPopulation.begin(), combinedPopulation.end(),
        [this](const NumArr& a, const NumArr& b) {
            return calculateFitness(a) > calculateFitness(b);
        });

    population.assign(combinedPopulation.begin(), combinedPopulation.begin() + elitismCount);
    population.resize(combinedPopulation.size() - elitismCount);
}


void Graph::evolve() {
    const int numGenerations = 100;
    for (int generation = 0; generation < numGenerations; ++generation) {
        selection();
        crossover();
        mutation();
        replacePopulation();
    }
}

NumArr Graph::getBestChromosome() {
    return population[0];
}

NumArr Graph::genetic() {
    initializePopulation();
    evolve();
    return getBestChromosome();
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