#include "./Graph.hpp"

void Graph::getGraphWeights() {
    std::cout << "Enter node1 node2 and weight in this format\n\n";
    for (int i = 0; i < edgeCount; ++i) {
        std::string ans;
        std::cout << std::endl << i << " [node1] [node2] [weight]: ";
        std::getline(std::cin, ans);
        std::istringstream iss(ans);
        std::string word;
        int ind = 0;
        int weight;
        std::pair<int, int> neighborNodes = { 0, 0 };
        while (iss >> word) {
            switch (ind) {
            case 0: {
                neighborNodes.first = std::stoi(word);
                break;
            }
            case 1: {
                neighborNodes.second = std::stoi(word);
                break;
            }
            case 2: {
                weight = std::stoi(word);
                break;
            }
            default: throw std::runtime_error("Too meny input values. Not valid!.");
            }
            ++ind;
        }
        graphWeights[neighborNodes] = weight;
    }
};

void Graph::getNeighborMatrix() {
    neighborMatrix = std::vector< std::vector< int > >( nodeCount, std::vector< int >( nodeCount, 0 ) );

    for (auto &pair: graphWeights)
    {
        int node1 = pair.first.first, node2 = pair.first.second, weight = pair.second;
        neighborMatrix[node1][node2] = weight;
        neighborMatrix[node2][node1] = weight;
    }
};



void Graph::printMatrix()
{
    std::cout << "  ";
    for (int i = 0; i < neighborMatrix.size(); ++i)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    for (int i = 0; i < neighborMatrix.size(); ++i)
    {
        std::cout << i << " ";
        for (int j = 0; j < neighborMatrix.size(); ++j)
        {
            std::cout << neighborMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void Graph::inputGraph()
{
    std::cout << "Input Node and Edge count: "; std::cin >> nodeCount >> edgeCount;

    getGraphWeights();
    getNeighborMatrix();
}


NumArr Graph::astar() {
    int start, target;
    std::cout << "enter start and target nodes"; std::cin >> start >> target;
    int n = neighborMatrix.size();
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
            if (neighborMatrix[current.vertex][neighbor] != 0) {
                int newCost = current.cost + neighborMatrix[current.vertex][neighbor];

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
    int n = neighborMatrix.size();
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
            if (neighborMatrix[current.vertex][neighbor] != 0) {
                int newDistance = current.cost + neighborMatrix[current.vertex][neighbor];

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
        for (int vertex = 0; vertex < neighborMatrix.size(); ++vertex) {
            std::cout << neighborMatrix.size() << "/" << vertex << std::endl;
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
        if (currentVertex >= neighborMatrix.size() || nextVertex >= neighborMatrix.size()) {
            pathLength += maxPathLength;
        }
        else {
            if (neighborMatrix[currentVertex][nextVertex] < maxPathLength - pathLength) {
                pathLength += neighborMatrix[currentVertex][nextVertex];
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

NumArr Graph::getLongestPath() {
    NumArr path;
    for (auto &weight:graphWeights) {
        weight.second *=-1;
    }
    getNeighborMatrix();
    path = astar();
    return path;
}
