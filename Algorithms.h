//
// Created by Letícia Coelho on 03/05/2024.
//

#ifndef DA_PROJECT2_ALGORITHMS_H
#define DA_PROJECT2_ALGORITHMS_H
#include <iostream>
#include <vector>
#include "VertexEdge.h"
using namespace std;

class Algorithms {
public:
    Algorithms(const Graph* g) : graph(g) {}
    void backtrackingAlgorithm(const Graph& graph, const std::string& graphFile);
    double getDistance(const Graph& graph, int sourceId, int destId) const;
    double tspBacktracking(const Graph& graph, unsigned currentVertex, unsigned count, std::vector<unsigned>& currentPath, std::vector<bool>& visited, double currentCost, double& minCost, std::vector<unsigned>& bestPath) const;

    double tsp2Approximation(int startId, std::vector<int>& tspPath) const;
    void preorderTraversal(int currentId, const std::unordered_map<int, std::vector<int>>& mstAdjList, std::unordered_set<int>& visited, std::vector<int>& path) const;

    void dfsTraversal(int u, int parent, const std::vector<std::vector<int>>& adjList,
                                  std::vector<bool>& visited, std::vector<int>& tspPath) const;

    std::pair<double, std::vector<int>> nearestNeighbor(const Graph& graph, int startNode);

    std::pair<double, std::vector<int>> twoOpt(const Graph& graph, const std::vector<int>& initialTour);

    double tSP2OptImprovement(vector<unsigned int>& path);

    std::pair<double, std::vector<int>> simulatedAnnealing(const Graph& graph, const std::vector<int>& initialTour);

    void printTour(const std::vector<int>& tour) const;

private:
    const Graph* graph;
};



#endif //DA_PROJECT2_ALGORITHMS_H
