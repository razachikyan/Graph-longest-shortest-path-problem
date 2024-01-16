#include "./Graph.hpp"

void Graph::getGraphWeights() {
    for (int i = 0; i < edgeCount; ++i) {
        std::string ans;
        std::cout << std::endl << i << " Enter node1 node2 and weight in this format [node1] [node2] [weight]";
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
    for (int i = 0; i < nodeCount; ++i) {
        for (int j = 0; j < nodeCount; ++j) {
            auto weight = graphWeights.find(std::make_pair(i, j));
            neighborMatrix[i][j] = (weight != graphWeights.end()) ? weight->second : 0;
        }
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

    neighborMatrix = std::vector< std::vector< int > >(nodeCount, std::vector< int >(nodeCount, 0));

    for (int i = 0; i < edgeCount; ++i)
    {
        int node1, node2, weight;
        std::cout << "Input node_1, node_2 and weight: "; std::cin >> node1 >> node2 >> weight;
        neighborMatrix[node1][node2] = weight;
        neighborMatrix[node2][node1] = weight;
    }
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

NumArr Graph::getLongestPath(int populationSize, double crossoverRate, double mutationRate, int numGenerations) {
    auto initializePopulation = [this]() -> std::vector<NumArr> {
        Population population;
        for (int i = 0; i < populationSize; ++i) {
            NumArr chromosome;
            for (int vertex = 0; vertex < neighborMatrix.size(); ++vertex) {
                chromosome.push_back(vertex);
            }
            std::random_shuffle(chromosome.begin(), chromosome.end());
            population.push_back(chromosome);
        }
        return population;
        };

    auto calculateFitness = [this](const NumArr& chromosome) -> double {
        int pathLength = 0;
        for (size_t i = 0; i < chromosome.size() - 1; ++i) {
            int currentVertex = chromosome[i];
            int nextVertex = chromosome[i + 1];

            // Check if there is an edge between the current and next vertices in the neighborMatrix.
            if (currentVertex < neighborMatrix.size() && nextVertex < neighborMatrix.size()) {
                pathLength += neighborMatrix[currentVertex][nextVertex];
            }
            else {
                // Handle the case where the vertices are out of bounds.
                // You may want to add an appropriate penalty.
                // Here, we add a penalty equal to the maximum possible weight in the graph.
                pathLength += std::numeric_limits<int>::max();
            }
        }

        // The fitness is inversely proportional to the path length.
        // You might want to adjust this based on your specific problem.
        return 1.0 / static_cast<double>(pathLength + 1);
        };



    auto selection = [this](Population& population) {
        // Calculate total fitness of the population
        double totalFitness = 0.0;
        for (const auto& chromosome : population) {
            totalFitness += calculateFitness(chromosome);
        }

        // Create a vector to store the cumulative probabilities
        std::vector<double> cumulativeProbabilities;
        cumulativeProbabilities.reserve(population.size());

        // Calculate cumulative probabilities
        double cumulativeProbability = 0.0;
        for (const auto& chromosome : population) {
            double probability = calculateFitness(chromosome) / totalFitness;
            cumulativeProbability += probability;
            cumulativeProbabilities.push_back(cumulativeProbability);
        }

        // Select chromosomes for reproduction
        Population selectedChromosomes;
        for (int i = 0; i < populationSize; ++i) {
            double randomValue = static_cast<double>(rand()) / RAND_MAX; // Generate a random value between 0 and 1

            // Find the chromosome whose cumulative probability is greater than the random value
            auto selectedIt = std::upper_bound(cumulativeProbabilities.begin(), cumulativeProbabilities.end(), randomValue);

            // Determine the index of the selected chromosome
            int selectedIndex = static_cast<int>(std::distance(cumulativeProbabilities.begin(), selectedIt));

            // Add the selected chromosome to the new population
            selectedChromosomes.push_back(population[selectedIndex]);
        }

        // Replace the old population with the selected one
        population = selectedChromosomes;
        };


    auto crossover = [this](Population& population) {
        Population newPopulation;

        // Perform crossover for pairs of chromosomes
        for (size_t i = 0; i < population.size(); i += 2) {
            // Ensure we have at least two chromosomes remaining
            if (i + 1 < population.size()) {
                // Select two parent chromosomes
                const NumArr& parent1 = population[i];
                const NumArr& parent2 = population[i + 1];

                // Choose a random crossover point
                size_t crossoverPoint = rand() % (parent1.size() - 1) + 1; // Avoid crossover at the ends

                // Create two new offspring chromosomes using one-point crossover
                NumArr offspring1(parent1.begin(), parent1.begin() + crossoverPoint);
                offspring1.insert(offspring1.end(), parent2.begin() + crossoverPoint, parent2.end());

                NumArr offspring2(parent2.begin(), parent2.begin() + crossoverPoint);
                offspring2.insert(offspring2.end(), parent1.begin() + crossoverPoint, parent1.end());

                // Add the new offspring to the new population
                newPopulation.push_back(offspring1);
                newPopulation.push_back(offspring2);
            }
            else {
                // If there's an odd number of chromosomes, just add the last one as is
                newPopulation.push_back(population[i]);
            }
        }

        // Replace the old population with the new one
        population = newPopulation;
        };


    auto mutation = [this](Population& population) {
        const double mutationRate = 0.1; // Adjust the mutation rate as needed

        for (auto& chromosome : population) {
            // Randomly decide whether to apply mutation
            if ((static_cast<double>(rand()) / RAND_MAX) < mutationRate) {
                // Choose two distinct positions for swap mutation
                size_t position1 = rand() % chromosome.size();
                size_t position2;
                do {
                    position2 = rand() % chromosome.size();
                } while (position1 == position2);

                // Perform swap mutation
                std::swap(chromosome[position1], chromosome[position2]);
            }
        }
        };


    auto replacePopulation = [this](Population& population) {
        const size_t elitismCount = 2; // Adjust the number of top individuals to preserve

        // Combine the old and new populations
        Population combinedPopulation = population;

        // Sort the combined population based on fitness
        std::sort(combinedPopulation.begin(), combinedPopulation.end(),
            [this](const NumArr& a, const NumArr& b) {
                return calculateFitness(a) > calculateFitness(b);
            });

        // Select the top individuals (elitism)
        population.assign(combinedPopulation.begin(), combinedPopulation.begin() + elitismCount);

        // Clear the rest of the population
        population.resize(combinedPopulation.size() - elitismCount);
        };


    auto evolve = [this, &initializePopulation, &calculateFitness, &selection, &crossover, &mutation, &replacePopulation](int numGenerations) {
        Population population = initializePopulation();
        for (int generation = 0; generation < numGenerations; ++generation) {
            selection(population);
            crossover(population);
            mutation(population);
            replacePopulation(population);
        }
        };

    auto getBestChromosome = [this](const std::vector<NumArr>& population) -> NumArr {
        // Extract the best chromosome from the final population as the solution.
        return population[0];
        };

    // Initialize and evolve the population
    std::vector<NumArr> population = initializePopulation();
    evolve(numGenerations);

    // Retrieve the best chromosome from the final population.
    NumArr bestChromosome = getBestChromosome(population);

    // Return the best path found.
    return bestChromosome;
}