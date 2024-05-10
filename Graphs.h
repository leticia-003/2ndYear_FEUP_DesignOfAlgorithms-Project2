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

    bool addVertex(Vertex& vertex);

    void printGraph(const Graph* graph);

    double mstPrim(int startId);

    Edge* getEdge(int sourceId, int destId) const;

    std::vector<Vertex*> getVertices() const;

    bool isComplete() const;

    double getDistance(int sourceId, int destId) const;

    Graph createToyGraphs(const std::string& graphFile);

    Graph createExtraFullyConnectedGraph(const std::string& graphFile);

    Graph* buildRealWorldGraph(unsigned number);

    void buildRealWorldGraphEdges(unsigned number, Graph* graph);

    void buildRealWorldGraphNodes(unsigned number, Graph* graph);

    unsigned size() const;

protected:

    std::unordered_map<int, Vertex *> vertexMap;
};


#endif //DA_PROJECT2_GRAPHS_H
