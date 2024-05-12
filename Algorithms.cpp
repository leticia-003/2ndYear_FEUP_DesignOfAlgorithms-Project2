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


void Algorithms::preorderTraversal(int currentId, const std::unordered_map<int, std::vector<int>>& mstAdjList, std::unordered_set<int>& visited, std::vector<int>& path) const {
    visited.insert(currentId);
    path.push_back(currentId);

    auto it = mstAdjList.find(currentId);
    if (it != mstAdjList.end()) {
        for (int neighborId : it->second) {
            if (visited.find(neighborId) == visited.end()) {
                preorderTraversal(neighborId, mstAdjList, visited, path);
            }
        }
    }
}

double Algorithms::tsp2Approximation(int startId, std::vector<int>& tspPath) const {
    // Get the MST edges
    std::vector<std::pair<unsigned, unsigned>> mST;
    double mstCost = graph->mstPrim(startId, mST);

    if (mstCost == 0.0) {
        std::cerr << "Unable to generate MST or the graph is empty." << std::endl;
        return 0.0;
    }

    // Map to store the adjacency list of the MST
    std::unordered_map<int, std::vector<int>> mstAdjList;
    for (const auto& edge : mST) {
        mstAdjList[edge.first].push_back(edge.second);
        mstAdjList[edge.second].push_back(edge.first);
    }

    // Traverse the MST to generate the TSP path
    std::unordered_set<int> visited;
    dfsTraversal(startId, startId, mstAdjList, visited, tspPath);

    // Add the distance from the last vertex back to the starting vertex
    tspPath.push_back(startId);

    // Calculate the total cost of the TSP path
    double tspCost = 0.0;
    for (size_t i = 0; i < tspPath.size() - 1; ++i) {
        // Find the distance between tspPath[i] and tspPath[i + 1]
        Vertex* u = graph->findVertex(tspPath[i]);
        Vertex* v = graph->findVertex(tspPath[i + 1]);

        if (u && v) {
            const Edge* edge = u->getEdge(v->getId());
            if (edge) {
                tspCost += edge->getDistance();
            } else {
                // If the edge is not found, calculate the distance using haversine formula
                double distance = graph->haversine(u->getLatitude(), u->getLongitude(),
                                                   v->getLatitude(), v->getLongitude());
                std::cout << "Using haversine to calculate distance between " << u->getId() << " and " << v->getId() << ": " << distance << std::endl;
                tspCost += distance;
            }
        }
    }

    std::cout << "TSP path: ";
    for (int vertex : tspPath) {
        std::cout << vertex << " -> ";
    }

    std::cout << "TSP cost: " << tspCost << std::endl;

    return tspCost;
}

void Algorithms::dfsTraversal(int u, int parent, const std::unordered_map<int, std::vector<int>>& adjList,
                              std::unordered_set<int>& visited, std::vector<int>& tspPath) const {
    visited.insert(u);
    tspPath.push_back(u);

    // Sort the adjacent vertices in increasing order of their IDs
    std::vector<int> sortedAdj = adjList.at(u);
    std::sort(sortedAdj.begin(), sortedAdj.end());

    // Visit the adjacent vertices in the sorted order
    for (int v : sortedAdj) {
        if (v != parent && visited.find(v) == visited.end()) {
            dfsTraversal(v, u, adjList, visited, tspPath);
        }
    }
}

