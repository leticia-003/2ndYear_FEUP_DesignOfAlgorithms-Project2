//
// Created by Letícia Coelho on 03/05/2024.
//

#ifndef DA_PROJECT2_ALGORITHMS_H
#define DA_PROJECT2_ALGORITHMS_H
#include <iostream>
#include <vector>
#include "VertexEdge.h"


class Algorithms {
public:
    Algorithms(const Graph* g) : graph(g) {}
    void backtrackingAlgorithm(const Graph& graph, const std::string& graphFile);
    double getDistance(const Graph& graph, int sourceId, int destId) const;
    double tspBacktracking(const Graph& graph, unsigned currentVertex, unsigned count, std::vector<unsigned>& currentPath, std::vector<bool>& visited, double currentCost, double& minCost, std::vector<unsigned>& bestPath) const;

    double tsp2Approximation(int startId, std::vector<int>& tspPath) const;
    void preorderTraversal(int currentId, const std::unordered_map<int, std::vector<int>>& mstAdjList, std::unordered_set<int>& visited, std::vector<int>& path) const;
    void eulerianTour(int currentId, const std::unordered_map<int, std::vector<int>>& mstAdjList, std::unordered_set<int>& visited, std::vector<int>& tour) const;

private:
    const Graph* graph;
};



#endif //DA_PROJECT2_ALGORITHMS_H
