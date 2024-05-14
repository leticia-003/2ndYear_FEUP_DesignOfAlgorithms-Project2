//
// Created by Letícia Coelho on 28/04/2024.
//

#ifndef DA_PROJECT2_GRAPHS_H
#define DA_PROJECT2_GRAPHS_H

#include <iostream>
#include "VertexEdge.h"
#include <unordered_map>
#include <unordered_set>
using namespace std;


class Graph {
public:
    ~Graph();

    const std::vector<Vertex*>& getVertices() const;

    Vertex * findVertex(const int &id) const;

    bool addVertex(Vertex& vertex);

    void printGraph(const Graph& graph);

    Vertex* getVertex(unsigned id) const;

    Edge* findEdge(unsigned first, unsigned second);

    double getDistanceOrHaversine(int sourceId, int destId) const;

    std::vector<Edge*> getEdges(int sourceId) const;

    bool parseNodesFile(const std::string& graphDirectory, Graph& graph);

    double mstPrim(int startId, std::vector<std::pair<unsigned, unsigned>>& mST) const;

    Edge* getEdge(int sourceId, int destId) const;

    bool isComplete() const;

    double getDistance(int sourceId, int destId) const;

    double haversine(double lat1, double lon1, double lat2, double lon2) const;

    void dfsTree(unsigned startId, std::vector<unsigned>& tree) const;

    void dfsHelper(unsigned currentId, std::unordered_set<unsigned>& visited, std::vector<unsigned>& tree) const;

    Graph createToyGraphs(const std::string& graphFile);

    Graph createExtraFullyConnectedGraph(const std::string& graphFile);

    Graph createRealWorldGraphs(const std::string& graphFile);

    double tSP2OptImprovement(std::vector<int>& path);

    unsigned size() const;

protected:

    std::vector<Vertex *> vertexSet;
};


#endif //DA_PROJECT2_GRAPHS_H
