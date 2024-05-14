//
// Created by Letícia Coelho on 03/05/2024.
//
#include "Graphs.h"
#include "Algorithms.h"
#include <iostream>
#include <vector>
#include <limits>
#include <iomanip>
using namespace std;

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

    // Determine the size of the graph to initialize the adjacency list
    int numVertices = graph->getVertices().size();

    // Vector to store the adjacency list of the MST
    std::vector<std::vector<int>> mstAdjList(numVertices);
    for (const auto& edge : mST) {
        mstAdjList[edge.first].push_back(edge.second);
        mstAdjList[edge.second].push_back(edge.first);
    }

    // Vector to store visited nodes
    std::vector<bool> visited(numVertices, false);

    // Traverse the MST to generate the TSP path
    dfsTraversal(startId, -1, mstAdjList, visited, tspPath);

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

    return tspCost;
}


void Algorithms::dfsTraversal(int u, int parent, const std::vector<std::vector<int>>& adjList,
                              std::vector<bool>& visited, std::vector<int>& tspPath) const {
    visited[u] = true;
    tspPath.push_back(u);

    // Get the adjacent vertices
    std::vector<int> sortedAdj = adjList[u];

    // Visit the adjacent vertices
    for (int v : sortedAdj) {
        if (v != parent && !visited[v]) {
            dfsTraversal(v, u, adjList, visited, tspPath);
        }
    }
}

std::pair<double, std::vector<int>> Algorithms::nearestNeighbor(const Graph& graph, int startNode) {
    std::vector<int> tour;
    double totalDistance = 0.0;
    int numNodes = graph.getVertices().size();

    std::vector<bool> visited(numNodes, false);
    int current = startNode;
    visited[current] = true;
    tour.push_back(current);

    for (int i = 0; i < numNodes - 1; ++i) {
        int nearestNeighbor = -1;
        double minDistance = std::numeric_limits<double>::max();
        for (int j = 0; j < numNodes; ++j) {
            if (!visited[j]) {
                double distance = graph.getDistance(current, j);
                if (distance < minDistance) {
                    minDistance = distance;
                    nearestNeighbor = j;
                }
            }
        }
        totalDistance += minDistance;
        current = nearestNeighbor;
        visited[current] = true;
        tour.push_back(current);
    }

    // Return to the starting node
    totalDistance += graph.getDistance(current, startNode);
    tour.push_back(startNode);

    return std::make_pair(totalDistance, tour);
}

double Algorithms::tSP2OptImprovement(const Graph& graph,std::vector<int>& path) {
    double currDistance = 0;
    for (int i = 0; i < path.size() - 1; i++)
        currDistance += graph.findEdge(path[i], path[i + 1])->getDistance();

    double bestDistance = currDistance;

    bool found = true;
    while (found) {
        found = false;

        for (int i = 1; i < path.size() - 2; i++) {
            for (int j = i + 1; j < path.size() - 1; j++) {

                std::vector<int> newPath = path;

                currDistance -= graph.findEdge(newPath[i - 1], newPath[i])->getDistance();
                currDistance -= graph.findEdge(newPath[j], newPath[j + 1])->getDistance();

                if (j != i + 1) { // Non-consecutive nodes
                    currDistance -= graph.findEdge(newPath[i], newPath[i + 1])->getDistance();
                    currDistance -= graph.findEdge(newPath[j - 1], newPath[j])->getDistance();
                }

                std::swap(newPath[i], newPath[j]);

                currDistance += graph.findEdge(newPath[i - 1], newPath[i])->getDistance();
                currDistance += graph.findEdge(newPath[j], newPath[j + 1])->getDistance();

                if (j != i + 1) { // Non-consecutive nodes
                    currDistance += graph.findEdge(newPath[i], newPath[i + 1])->getDistance();
                    currDistance += graph.findEdge(newPath[j - 1], newPath[j])->getDistance();
                }

                if (currDistance < bestDistance) {
                    path = newPath;
                    bestDistance = currDistance;
                    found = true;
                    break;
                }
                else currDistance = bestDistance;
            }
        }
        if (found) break;
    }

    std::cout << "| 2-OPT Tour: ";

    for (int i = 0; i < path.size() - 1; i++) {
        std::cout << path[i] << " -> ";
    }
    std::cout << path.back(); // Print the last node without the arrow
    std::cout << std::endl;

    return bestDistance;
}


void Algorithms::printTour(const std::vector<int>& tour) const {
    for (size_t i = 0; i < tour.size(); ++i) {
        std::cout << tour[i];
        if (i != tour.size() - 1) // If not the last node
            std::cout << " -> ";
    }
    std::cout << std::endl;
}
