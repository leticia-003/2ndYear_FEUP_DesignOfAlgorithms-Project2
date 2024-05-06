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
    void backtrackingAlgorithm(const Graph& graph);
    double getDistance(const Graph& graph, int sourceId, int destId) const;
    double tspBacktracking(const Graph& graph, unsigned currentVertex, unsigned count, std::vector<unsigned>& currentPath, std::vector<bool>& visited, double currentCost, double& minCost, std::vector<unsigned>& bestPath) const;


    };



#endif //DA_PROJECT2_ALGORITHMS_H
