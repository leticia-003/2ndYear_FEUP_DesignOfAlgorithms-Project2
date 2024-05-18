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

/**
 * @class Graph
 * @brief A class representing a graph with various utility methods for graph operations.
 */
class Graph {
public:
    /**
     * @brief Destructor for the Graph class.
     */
    ~Graph();

    /**
     * @brief Gets the vertices of the graph.
     * @return A constant reference to the vector of vertices.
     */
    const std::vector<Vertex*>& getVertices() const;

    /**
     * @brief Finds a vertex in the graph by its ID.
     * @param id The ID of the vertex to find.
     * @return Pointer to the vertex if found, nullptr otherwise.
     */
    Vertex * findVertex(const int &id) const;

    /**
     * @brief Adds a vertex to the graph.
     * @param vertex The vertex to add.
     * @return True if the vertex was added, false otherwise.
     */
    bool addVertex(Vertex& vertex);

    /**
     * @brief Prints the graph.
     * @param graph The graph to print.
     */
    void printGraph(const Graph& graph);

    /**
     * @brief Finds an edge between two vertices.
     * @param first The ID of the first vertex.
     * @param second The ID of the second vertex.
     * @return Pointer to the edge if found, nullptr otherwise.
     */
    Edge* findEdge(unsigned first, unsigned second) const;

    /**
     * @brief Gets the edges originating from a given vertex.
     * @param sourceId The ID of the source vertex.
     * @return A vector of pointers to the edges.
     */
    std::vector<Edge*> getEdges(int sourceId) const;

    /**
     * @brief Parses a file containing nodes to create a graph.
     * @param graphDirectory The directory of the graph file.
     * @param graph The graph to populate.
     * @return True if parsing was successful, false otherwise.
     */
    bool parseNodesFile(const std::string& graphDirectory, Graph& graph);

    /**
     * @brief Calculates the Minimum Spanning Tree (MST) using Prim's algorithm.
     * @param startId The starting vertex ID for Prim's algorithm.
     * @param mST A vector to store the MST edges.
     * @return The total cost of the MST.
     */
    double mstPrim(int startId, std::vector<std::pair<unsigned, unsigned>>& mST) const;

    /**
     * @brief Gets the distance between two vertices.
     * @param sourceId The ID of the source vertex.
     * @param destId The ID of the destination vertex.
     * @return The distance between the source and destination vertices.
     */
    double getDistance(int sourceId, int destId) const;

    /**
     * @brief Calculates the haversine distance between two geographical points.
     * @param lat1 Latitude of the first point.
     * @param lon1 Longitude of the first point.
     * @param lat2 Latitude of the second point.
     * @param lon2 Longitude of the second point.
     * @return The haversine distance between the two points.
     */
    double haversine(double lat1, double lon1, double lat2, double lon2) const;

    /**
     * @brief Helper function for depth-first search (DFS).
     * @param currentId The current vertex ID in the DFS.
     * @param visited A set of visited vertices.
     * @param tree A vector to store the DFS tree.
     */
    void dfsHelper(unsigned currentId, std::unordered_set<unsigned>& visited, std::vector<unsigned>& tree) const;

    /**
     * @brief Creates a toy graph from a file.
     * @param graphFile The file containing the toy graph.
     * @return The created toy graph.
     */
    Graph createToyGraphs(const std::string& graphFile);

    /**
     * @brief Creates an extra fully connected graph from a file.
     * @param graphFile The file containing the graph.
     * @return The created fully connected graph.
     */
    Graph createExtraFullyConnectedGraph(const std::string& graphFile);

    /**
     * @brief Creates a real-world graph from a file.
     * @param graphFile The file containing the real-world graph.
     * @return The created real-world graph.
     */
    Graph createRealWorldGraphs(const std::string& graphFile);

    /**
     * @brief Gets the size of the graph.
     * @return The number of vertices in the graph.
     */
    unsigned size() const;

protected:
    std::vector<Vertex *> vertexSet; ///< The set of vertices in the graph.
};

#endif //DA_PROJECT2_GRAPHS_H
