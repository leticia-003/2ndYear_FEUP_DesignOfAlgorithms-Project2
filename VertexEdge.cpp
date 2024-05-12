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

double Vertex::getLatitude() const {
    return latitude;
}

double Vertex::getLongitude() const {
    return longitude;
}

Edge * Vertex::addEdge(Vertex* dest, double distance) {
    Edge* edge = new Edge(this, dest, distance);
    adj.push_back(edge);
    return edge;
}

// Get an edge given the destination vertex's ID
Edge* Vertex::getEdge(int destId) {
    for (Edge* edge : adj) {
        if (edge->getDest()->getId() == destId) {
            return edge;
        }
    }
    return nullptr; // Edge not found
}

Vertex* Vertex::getParent() const {
    return parent;
}

void Vertex::setParent(Vertex* p) {
    parent = p;
}


/************************** Edge ***************************/

Edge::Edge(Vertex* orig, Vertex* dest, double distance) : orig(orig), dest(dest), distance(distance) {}

Vertex * Edge::getOrig() const {
    return this->orig;
}

Vertex * Edge::getDest() const {
    return this->dest;
}

double Edge::getDistance() const {
    return distance;
}