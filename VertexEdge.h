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

class Vertex {
public:

    Vertex(int id);

    std::vector<Edge *> getAdj() const;

    Edge * addEdge(Vertex* dest, int distance);

    int getId() const;

protected:
    int id;

    std::vector<Edge *> adj;

};

/********************** Edge ****************************/

/**
 * @brief Class that defines a graph's edge
 */

class Edge {
public:

    Edge(Vertex* orig, Vertex* dest, double distance);

    Vertex *getDest() const;

protected:

    Vertex * dest;

    Vertex *orig;

    double distance;

};


#endif //DA_PROJECT2_VERTEXEDGE_H
