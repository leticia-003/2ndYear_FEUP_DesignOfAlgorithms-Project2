//
// Created by Letícia Coelho on 29/04/2024.
//

#ifndef DA_PROJECT2_VERTEXEDGE_H
#define DA_PROJECT2_VERTEXEDGE_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

class Edge;

#define INF std::numeric_limits<double>::max()

/************************* Vertex **************************/

/**
 * @class Vertex
 * @brief A class representing a vertex in a graph.
 */
class Vertex {
public:
    /**
     * @brief Constructs a Vertex with a given ID.
     * @param id The ID of the vertex.
     */
    Vertex(int id);

    /**
     * @brief Gets the adjacency list of the vertex.
     * @return A vector of pointers to the edges.
     */
    std::vector<Edge *> getAdj() const;

    /**
     * @brief Adds an edge to the vertex.
     * @param dest The destination vertex of the edge.
     * @param distance The distance of the edge.
     * @return Pointer to the added edge.
     */
    Edge * addEdge(Vertex* dest, double distance);

    /**
     * @brief Gets the ID of the vertex.
     * @return The ID of the vertex.
     */
    int getId() const;

    /**
     * @brief Gets an edge to a specific destination vertex.
     * @param destId The ID of the destination vertex.
     * @return Pointer to the edge if found, nullptr otherwise.
     */
    Edge* getEdge(int destId);

    /**
     * @brief Gets the latitude of the vertex.
     * @return The latitude of the vertex.
     */
    double getLatitude() const;

    /**
     * @brief Gets the longitude of the vertex.
     * @return The longitude of the vertex.
     */
    double getLongitude() const;

    /**
     * @brief Sets the latitude of the vertex.
     * @param lat The latitude to set.
     */
    void setLatitude(double lat);

    /**
     * @brief Sets the longitude of the vertex.
     * @param lon The longitude to set.
     */
    void setLongitude(double lon);

protected:
    int id; ///< The ID of the vertex.

    std::vector<Edge *> adj; ///< The adjacency list of the vertex.

    double latitude; ///< The latitude of the vertex.

    double longitude; ///< The longitude of the vertex.

    Vertex* parent; ///< The parent vertex in the path.
};

/********************** Edge ****************************/

/**
 * @class Edge
 * @brief Class that defines a graph's edge.
 */
class Edge {
public:
    /**
     * @brief Constructs an Edge with given origin, destination, and distance.
     * @param orig The origin vertex of the edge.
     * @param dest The destination vertex of the edge.
     * @param distance The distance of the edge.
     */
    Edge(Vertex* orig, Vertex* dest, double distance);

    /**
     * @brief Gets the destination vertex of the edge.
     * @return Pointer to the destination vertex.
     */
    Vertex *getDest() const;

    /**
     * @brief Gets the origin vertex of the edge.
     * @return Pointer to the origin vertex.
     */
    Vertex * getOrig() const;

    /**
     * @brief Gets the distance of the edge.
     * @return The distance of the edge.
     */
    double getDistance() const;

protected:
    Vertex * dest; ///< The destination vertex of the edge.

    Vertex *orig; ///< The origin vertex of the edge.

    double distance; ///< The distance of the edge.
};

#endif //DA_PROJECT2_VERTEXEDGE_H
