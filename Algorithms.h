//
// Created by Letícia Coelho on 03/05/2024.
//

#ifndef DA_PROJECT2_ALGORITHMS_H
#define DA_PROJECT2_ALGORITHMS_H
#include <iostream>
#include "VertexEdge.h"


class Algorithms {
public:
    double tspBacktracking(std::vector<Vertex*>& graph, std::vector<int>& bestPath, double& elapsedTime);
private:
    void tspUtil(int currentVertex, int visitedCount, std::vector<bool>& visited, double currentCost, double& minCost, std::vector<int>& currentPath, std::vector<int>& bestPath, std::vector<Vertex*>& graph);

};



#endif //DA_PROJECT2_ALGORITHMS_H
