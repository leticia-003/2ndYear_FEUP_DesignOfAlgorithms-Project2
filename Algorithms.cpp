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

void Algorithms::triangularApproximationTSP(const Graph& graph, const std::string& graphFile) {
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<bool> visited(graph.size(), false);
    std::vector<unsigned> path;
    double totalCost = 0.0;
    unsigned current = 0; // Start at node 0
    path.push_back(current);
    visited[current] = true;

    // Nearest neighbor heuristic
    for (unsigned i = 1; i < graph.size(); ++i) {
        double nearestDistance = std::numeric_limits<double>::max();
        unsigned nearestVertex = 0;

        for (unsigned j = 0; j < graph.size(); ++j) {
            if (!visited[j]) {
                double distance = getDistance(graph, current, j);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestVertex = j;
                }
            }
        }

        if (nearestDistance == std::numeric_limits<double>::max()) {
            std::cerr << "No valid path found from " << current << std::endl;
            break; // Break out of the loop if no valid path can be found (isolated node or disconnected graph)
        }

        visited[nearestVertex] = true;
        path.push_back(nearestVertex);
        totalCost += nearestDistance;
        current = nearestVertex;
    }

    // Return to the starting node
    double returnCost = getDistance(graph, current, 0);
    totalCost += returnCost;
    path.push_back(0); // Complete the circuit by returning to the start

    auto end = std::chrono::high_resolution_clock::now();

    // Output the results
    std::cout << "Algorithm: TSP Triangular Approximation" << std::endl;
    std::cout << "Graph: ";
    std::string graphName = graphFile.substr(0, graphFile.find_last_of('.'));
    std::cout << graphName << std::endl;
    std::cout << "Time: ";
    std::cout << (end - start) / std::chrono::milliseconds(1) << " ms" << std::endl;
    std::cout << "Total Cost: ";
    std::cout << totalCost << std::endl;
    std::cout << "Path: ";

    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << std::setfill(' ') << std::left << path[i];
        if (i != path.size() - 1)
            std::cout << " -> ";
    }
    std::cout << std::endl;
}

