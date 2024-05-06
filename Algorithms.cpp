//
// Created by Letícia Coelho on 03/05/2024.
//

#include "Graphs.h"
#include "Algorithms.h"
#include <iostream>
#include <vector>
#include <limits>

void Algorithms::backtrackingAlgorithm(const Graph& graph) {
    std::vector<unsigned> path, bestPath;
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<bool> visited(graph.size(), false);

    std::cout << "Starting backtracking..." << std::endl;  // Debug statement

    double minCost = std::numeric_limits<double>::max();
    double cost = tspBacktracking(graph, 0, 1, path, visited, 0.0, minCost, bestPath); // count starts at 1 because vertex 0 is counted as visited
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Backtracking completed." << std::endl;  // Debug statement

    std::cout << "\nAccording to the backtracking algorithm, the minimal cost circuit visiting all nodes is:" << std::endl;
    for (unsigned p : bestPath) {
        std::cout << " -> " << p;
    }
    std::cout << "\nThe cost of the circuit is " << minCost << "." << std::endl;
    std::cout << "\nThe algorithm took approximately " << (end - start) / std::chrono::milliseconds(1) << " milliseconds to execute." << std::endl;
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
