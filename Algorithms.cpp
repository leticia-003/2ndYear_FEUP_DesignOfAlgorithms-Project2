//
// Created by Letícia Coelho on 03/05/2024.
//
#include "Graphs.h"
#include "Algorithms.h"
#include <iostream>
#include <vector>
#include <limits>
#include <iomanip>

void Algorithms::backtrackingAlgorithm(const Graph& graph, const std::string& graphFile) {
    std::vector<unsigned> path, bestPath;
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<bool> visited(graph.size(), false);

    double minCost = std::numeric_limits<double>::max();
    double cost = tspBacktracking(graph, 0, 1, path, visited, 0.0, minCost, bestPath); // count starts at 1 because vertex 0 is counted as visited
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Algorithm: TSP Backtracking" << std::endl;
    std::cout << "Graph: ";
    std::string graphName = graphFile.substr(0, graphFile.find_last_of('.'));
    std:: cout << graphName << std::endl;
    std::cout << "Time: ";
    std::cout << (end - start) / std::chrono::milliseconds(1) << " ms" << std::endl;
    std::cout << "Minimal Cost: ";
    std::cout << minCost << std::endl;
    std::cout << "Corresponding Path: ";

    // Center-align the path
    for (size_t i = 0; i < bestPath.size(); ++i) {
        std::cout << std::setfill(' ') << std::left << bestPath[i];
        if (i != bestPath.size() - 1)
            std::cout << " -> ";
    }
}


double Algorithms::getDistance(const Graph& graph, int sourceId, int destId) const {
    Vertex* sourceVertex = graph.findVertex(sourceId);
    Vertex* destVertex = graph.findVertex(destId);
    if (sourceVertex && destVertex) {
        Edge* edge = sourceVertex->getEdge(destId);
        if (edge) {
            return edge->getDistance();
        }
    }
    return std::numeric_limits<double>::max(); // If there's no edge, return infinity or some large value
}

double Algorithms::tspBacktracking(const Graph& graph, unsigned currentVertex, unsigned count, std::vector<unsigned>& currentPath, std::vector<bool>& visited, double currentCost, double& minCost, std::vector<unsigned>& bestPath) const {
    // Include the current vertex in the path
    currentPath.push_back(currentVertex);
    visited[currentVertex] = true;

    if (count == graph.size()) {
        // Check return to the starting vertex
        double returnCost = getDistance(graph, currentVertex, 0);
        currentCost += returnCost;

        if (currentCost < minCost && returnCost != std::numeric_limits<double>::max()) {
            minCost = currentCost;
            bestPath = currentPath;
            bestPath.push_back(0);   // Include the return to the starting vertex
        }

        // Unmark the current vertex as visited and remove it from the current path
        visited[currentVertex] = false;
        currentPath.pop_back();

        return currentCost;
    }

    double minPathCost = std::numeric_limits<double>::max();
    for (unsigned i = 0; i < graph.size(); ++i) {
        if (!visited[i] && i != currentVertex) {
            double distance = getDistance(graph, currentVertex, i);
            if (distance != std::numeric_limits<double>::max() && currentCost + distance < minCost) {
                double pathCost = tspBacktracking(graph, i, count + 1, currentPath, visited, currentCost + distance, minCost, bestPath);
                minPathCost = std::min(minPathCost, pathCost);
            }
        }
    }

    // Unmark the current vertex as visited and remove it from the current path
    visited[currentVertex] = false;
    currentPath.pop_back();

    return minPathCost;
}

void Algorithms::approximationAlgorithm(const Graph& graph, const std::string& graphFile) {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<unsigned> approxPath;
    std::vector<bool> visited(graph.size(), false);
    double approxCost = 0.0;

    // Start at vertex 0
    unsigned currentVertex = 0;
    visited[currentVertex] = true;
    approxPath.push_back(currentVertex);

    // Repeat until all vertices are visited
    while (approxPath.size() < graph.size()) {
        double minDistance = std::numeric_limits<double>::max();
        unsigned nextVertex = 0;

        // Find the nearest unvisited vertex
        for (unsigned i = 0; i < graph.size(); ++i) {
            if (!visited[i]) {
                double distance = getDistance(graph, currentVertex, i);
                if (distance < minDistance) {
                    minDistance = distance;
                    nextVertex = i;
                }
            }
        }

        // Move to the nearest unvisited vertex
        visited[nextVertex] = true;
        approxPath.push_back(nextVertex);
        approxCost += minDistance;
        currentVertex = nextVertex;
    }

    // Return to the starting vertex
    approxPath.push_back(0);
    approxCost += getDistance(graph, currentVertex, 0);

    auto end = std::chrono::high_resolution_clock::now();

    // Print the results
    std::cout << "Algorithm: Approximation Algorithm" << std::endl;
    std::cout << "Graph: " << graphFile << std::endl;
    std::cout << "Time: " << (end - start) / std::chrono::milliseconds(1) << " ms" << std::endl;
    std::cout << "Minimal Cost: " << approxCost << std::endl;
    std::cout << "Corresponding Path: ";
    for (size_t i = 0; i < approxPath.size(); ++i) {
        std::cout << std::setw(3) << std::setfill(' ') << std::left << approxPath[i];
        if (i != approxPath.size() - 1)
            std::cout << " -> ";
    }
    std::cout << std::endl;
}
