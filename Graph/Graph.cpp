#include "./Graph.hpp"

void Graph::getGraphWeights() {
    for (int i = 0; i < edgeCount; ++i) {
        std::pair<int, int> neighborNodes = { 0, 0 };
        int weight;
        std::cout << "Input node_1, node_2 and weight: "; std::cin >> neighborNodes.first >> neighborNodes.second >> weight;
        graphWeights[neighborNodes] = weight;
    }
};

void Graph::getNeighborMatrix() {
    graph = std::vector< std::vector< int > >(nodeCount, std::vector< int >(nodeCount, 0)); // defone from map
    for (auto& pair : graphWeights)
    {
        int node1 = pair.first.first, node2 = pair.first.second, weight = pair.second;
        graph[node1][node2] = weight;
        graph[node2][node1] = weight;
    }
};

void Graph::printGraph()
{
    std::cout << "  ";
    for (int i = 0; i < graph.size(); ++i)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    for (int i = 0; i < graph.size(); ++i)
    {
        std::cout << i << " ";
        for (int j = 0; j < graph.size(); ++j)
        {
            std::cout << graph[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void Graph::printPath(NumArr& path) {
    for (int i = 0; i < path.size(); ++i) {
        std::cout << path[i] << " ";
    }
    std::cout << std::endl;
}

void Graph::inputGraph()
{
    std::cout << "Input Node and Edge count: "; std::cin >> nodeCount >> edgeCount;

    getGraphWeights();
    getNeighborMatrix();
}

void Graph::readGraphFromFile(const std::string& path) {
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    int node1, node2, weight;
    while (inputFile >> node1 >> node2 >> weight) {
        graphWeights[std::make_pair(node1, node2)] = weight;
    }

    inputFile.close();

    nodeCount = 0;
    for (const auto& pair : graphWeights) {
        nodeCount = std::max(nodeCount, std::max(pair.first.first, pair.first.second));
    }
    nodeCount++;

    edgeCount = graphWeights.size();

    getNeighborMatrix();
    printGraph();
};

NumArr Graph::astar() {
    int start, target;
    std::cout << "enter start and target nodes"; std::cin >> start >> target;
    int n = graph.size();
    NumArr parent(n, -1);
    NumArr cost(n, INT_MAX);
    NumArr heuristic(n, 0);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    pq.push({ start, 0, 0 });
    cost[start] = 0;

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.vertex == target) {
            NumArr path;
            int node = target;

            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < n; ++neighbor) {
            if (graph[current.vertex][neighbor] != 0) {
                int newCost = current.cost + graph[current.vertex][neighbor];

                if (newCost < cost[neighbor]) {
                    cost[neighbor] = newCost;
                    parent[neighbor] = current.vertex;
                    pq.push({ neighbor, newCost, heuristic[neighbor] });
                }
            }
        }
    }

    return NumArr();
}

NumArr Graph::dijkstra() {
    int start, target;
    std::cout << "enter start and target nodes"; std::cin >> start >> target;
    int n = graph.size();
    NumArr parent(n, -1);
    NumArr distance(n, INF);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    pq.push({ start, 0 });
    distance[start] = 0;

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.vertex == target) {
            NumArr path;
            int node = target;

            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < n; ++neighbor) {
            if (graph[current.vertex][neighbor] != 0) {
                int newDistance = current.cost + graph[current.vertex][neighbor];

                if (newDistance < distance[neighbor]) {
                    distance[neighbor] = newDistance;
                    parent[neighbor] = current.vertex;
                    pq.push({ neighbor, newDistance });
                }
            }
        }
    }

    return NumArr();
}

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
        else {
            newPopulation.push_back(population[i]);
        }
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
    if(strategyType == "dijkstra") this->strategy = std::make_unique<DijkstraStrategy>();
    else if(strategyType == "astar") this->strategy = std::make_unique<AStarStrategy>();
    else throw std::runtime_error("Incorrect strategy type");
}

NumArr Graph::getShortestPath() {
    if(!this->strategy) throw std::runtime_error("Strategy novu");
    int start, target;
    std::cout << "enter start and target nodes"; std::cin >> start >> target;
    NumArr path = strategy->execute(graph, start, target);
}

void Graph::draw() {
    GVC_t *gvc;
    Agraph_t *aGraph;

    gvc = gvContext();

    aGraph = agopen("myGraph", Agundirected, NULL);

    Agnode_t *nodes[graph.size()];
    for (int i = 0; i < graph.size(); ++i) {
        char nodeName[5]; // 5 - Graph name length
        sprintf(nodeName, "%d",  i + 1);
        nodes[i] = agnode(aGraph, nodeName, 1);
    }

    for (int i = 0; i < graph.size(); ++i) {
        for (int j = i + 1; j < graph.size(); ++j) {
            if (graph[i][j] != 0) {
                agedge(aGraph, nodes[i], nodes[j], nullptr, graph[i][j]);
            }
        }
    }

    gvLayout(gvc, aGraph, "dot");
    gvRenderFilename(gvc, aGraph, "png", "output.png");
    gvFreeLayout(gvc, aGraph);
    agclose(aGraph);
    gvFreeContext(gvc);
}
