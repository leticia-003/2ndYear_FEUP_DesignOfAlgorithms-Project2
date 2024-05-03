//
// Created by Letícia Coelho on 03/05/2024.
//

#include <iostream>
#include "Algorithms.h"

double Algorithms::tspBacktracking(std::vector<Vertex*>& graph, std::vector<int>& bestPath, double& elapsedTime) {
    std::vector<bool> visited(graph.size(), false);
    std::vector<int> currentPath;
    double minCost = std::numeric_limits<double>::max();
    visited[0] = true;
    currentPath.push_back(0);

    auto start = std::chrono::high_resolution_clock::now();
    tspUtil(0, 1, visited, 0, minCost, currentPath, bestPath, graph);
    auto end = std::chrono::high_resolution_clock::now();
    elapsedTime = std::chrono::duration<double>(end - start).count();

    return minCost;
}

void Algorithms::tspUtil(int currentVertex, int visitedCount, std::vector<bool>& visited, double currentCost, double& minCost, std::vector<int>& currentPath, std::vector<int>& bestPath, std::vector<Vertex*>& graph) {
    if (visitedCount == graph.size()) {
        for (Edge* edge : graph[currentVertex]->getAdj()) {
            if (edge->getDest()->getId() == graph[0]->getId()) {
                currentPath.push_back(0);
                if (currentCost + edge->getDistance() < minCost) {
                    minCost = currentCost + edge->getDistance();
                    bestPath = currentPath;
                }
                currentPath.pop_back();
                return;
            }
        }
        return;
    }

    for (Edge* edge : graph[currentVertex]->getAdj()) {
        if (!visited[edge->getDest()->getId()]) {
            visited[edge->getDest()->getId()] = true;
            currentPath.push_back(edge->getDest()->getId());
            tspUtil(edge->getDest()->getId(), visitedCount + 1, visited, currentCost + edge->getDistance(), minCost, currentPath, bestPath, graph);
            currentPath.pop_back();
            visited[edge->getDest()->getId()] = false;
        }
    }
}



