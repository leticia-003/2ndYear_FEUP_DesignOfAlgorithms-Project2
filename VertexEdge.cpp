//
// Created by Letícia Coelho on 29/04/2024.
//

#include "VertexEdge.h"

/********************** Vertex *****************************/

Vertex::Vertex(int id) : id(id) {}


int Vertex::getId() const {
    return this->id;
}

std::vector<Edge *> Vertex::getAdj() const {
    return adj;
}

Edge * Vertex::addEdge(Vertex* dest, double distance) {
    Edge* edge = new Edge(this, dest, distance);
    adj.push_back(edge);
    return edge;
}


/************************** Edge ***************************/

Edge::Edge(Vertex* orig, Vertex* dest, double distance) : orig(orig), dest(dest), distance(distance) {}

Vertex * Edge::getDest() const {
    return this->dest;
}