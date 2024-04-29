#include "Graphs.h"
#include <cmath>
#include <fstream>
#include <sstream>

Graph::~Graph() {
    for (auto& pair : vertexMap) {
        delete pair.second; // Delete each vertex
    }
}

const std::unordered_map<int, Vertex *> & Graph::getVertexMap() const {
    return vertexMap;
}

Vertex * Graph::findVertex(const int &id) const {
    auto iter = vertexMap.find(id);
    if (iter != vertexMap.end()) {
        return iter->second;
    } else {
        return nullptr;
    }
}

bool Graph::addVertex(const Vertex& vertex) {
    if (findVertex(vertex.getId()) != nullptr)
        return false;
    vertexMap.insert({vertex.getId(), new Vertex(vertex)});
    return true;
}



// Function to calculate Haversine distance between two latitude-longitude points
double haversine(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371.0; // Earth radius in kilometers

    // Convert latitude and longitude from degrees to radians
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;

    // Haversine formula
    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;
    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1) * cos(lat2) * sin(dlon / 2.0) * sin(dlon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double distance = R * c;

    return distance;
}

std::vector<std::vector<Vertex*>> Graph::createToyGraphs(const std::string& graphFile) {
    std::vector<std::vector<Vertex*>> graphs;

    // Add ".csv" extension to the provided file name
    std::string filePath = "../Toy-Graphs/" + graphFile;

    std::ifstream infile(filePath);
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        exit(1);
    }

    std::vector<Vertex*> graph;
    std::string line;
    bool firstLine = true; // Flag to indicate if it's the first line

    while (std::getline(infile, line)) {
        if (firstLine) {
            // Skip the first line
            firstLine = false;
            continue;
        }

        std::istringstream iss(line);
        int source, dest;
        double distance;
        char comma;
        // Read source, destination, and weight
        if (!(iss >> source >> comma >> dest >> comma >> distance)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue; // Skip invalid lines
        }

        // Add or find vertices
        Vertex *sourceVertex = findVertex(source);
        if (!sourceVertex) {
            sourceVertex = new Vertex(source);
            addVertex(*sourceVertex);
        }
        Vertex *destVertex = findVertex(dest);
        if (!destVertex) {
            destVertex = new Vertex(dest);
            addVertex(*destVertex);
        }

        // Add the edge
        sourceVertex->addEdge(destVertex, distance);
    }
    infile.close();
    graphs.push_back(graph);

    return graphs;
}


void Graph::printGraph(const std::vector<Vertex*>& graph) {
    std::cout << "Graph:" << std::endl;
    for (const auto& vertex : graph) {
        std::cout << "Vertex " << vertex->getId() << ": ";
        for (const auto& edge : vertex->getAdj()) {
            std::cout << edge->getDest()->getId() << " ";
        }
        std::cout << std::endl;
    }
}
