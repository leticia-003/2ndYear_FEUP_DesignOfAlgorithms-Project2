//
// Created by Letícia Coelho on 28/04/2024.
//

#ifndef DA_PROJECT2_GRAPHS_H
#define DA_PROJECT2_GRAPHS_H

#include <iostream>
#include "VertexEdge.h"
#include <unordered_map>

class Graph {
public:
    ~Graph();
    const std::unordered_map<int, Vertex *> & getVertexMap() const;

    Vertex * findVertex(const int &id) const;

    bool addVertex(const Vertex& vertex);

    void printGraph(const std::vector<Vertex*>& graph);

    std::vector<std::vector<Vertex*>> createToyGraphs(const std::string& graphFile);

protected:

    std::unordered_map<int, Vertex *> vertexMap;
};


#endif //DA_PROJECT2_GRAPHS_H
