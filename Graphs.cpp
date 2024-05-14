#include "Graphs.h"
#include <cmath>
#include <fstream>
#include <queue>
#include <unordered_set>
#include <sstream>
using namespace std;

Graph::~Graph() {
    for (auto& vertex : vertexSet) {
        delete vertex; // Delete each vertex
    }
}

const std::vector<Vertex*>& Graph::getVertices() const {
    return vertexSet;
}

unsigned Graph::size() const {
    return vertexSet.size();
}


Vertex* Graph::findVertex(const int& id) const {
    for (auto vertex : vertexSet) {
        if (vertex->getId() == id) {
            return vertex;
        }
    }
    return nullptr;
}


bool Graph::addVertex(Vertex& vertex) {
    if (findVertex(vertex.getId())) {
        return false; // Vertex with the same ID already exists
    }
    vertexSet.push_back(&vertex);
    return true;
}

std::vector<Edge*> Graph::getEdges(int sourceId) const {
    Vertex* sourceVertex = findVertex(sourceId);
    if (!sourceVertex) {
        return {}; // Return empty vector if source vertex not found
    }
    return sourceVertex->getAdj();
}

double Graph::getDistanceOrHaversine(int sourceId, int destId) const {
    // First try to get the direct distance
    double directDistance = getDistance(sourceId, destId);
    if (directDistance != INF) {
        return directDistance;
    }

    // If no direct edge exists, calculate Haversine distance
    Vertex* sourceVertex = findVertex(sourceId);
    Vertex* destVertex = findVertex(destId);
    if (sourceVertex && destVertex) {
        return haversine(sourceVertex->getLatitude(), sourceVertex->getLongitude(),
                         destVertex->getLatitude(), destVertex->getLongitude());
    }

    return INF; // If either vertex is not found, return infinity
}

bool Graph::parseNodesFile(const std::string& graphDirectory, Graph& graph) {
    std::string filePath = "../Real-world Graphs/" + graphDirectory + "/nodes.csv";
    std::ifstream infile(filePath);
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return false;
    }

    std::string line;
    bool firstLine = true;

    while (std::getline(infile, line)) {
        if (firstLine) {
            firstLine = false;
            continue; // Skip the header line
        }

        std::istringstream iss(line);
        unsigned id;
        double longitude, latitude;
        char comma;

        if (!(iss >> id >> comma >> longitude >> comma >> latitude)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue; // Skip invalid lines
        }

        Vertex* vertex = graph.findVertex(id);
        if (!vertex) {
            vertex = new Vertex(id);
            graph.addVertex(*vertex);
        }

        vertex->setLatitude(latitude);
        vertex->setLongitude(longitude);
    }

    infile.close();
    return true;
}

double Graph::mstPrim(int startId, std::vector<std::pair<unsigned, unsigned>>& mST) const {
    std::priority_queue<std::pair<double, std::pair<unsigned, unsigned>>,
            std::vector<std::pair<double, std::pair<unsigned, unsigned>>>,
            std::greater<std::pair<double, std::pair<unsigned, unsigned>>>> pq;
    std::unordered_set<unsigned> visited;
    mST.clear();

    // Push the start vertex and its edges into the priority queue
    for (auto edge : getEdges(startId)) {
        pq.push({edge->getDistance(), {startId, edge->getDest()->getId()}});
    }
    visited.insert(startId);

    double totalCost = 0.0;
    unsigned numVertices = vertexSet.size();

    while (!pq.empty() && visited.size() < numVertices) {
        auto [cost, edge] = pq.top();
        pq.pop();
        unsigned src = edge.first;
        unsigned dest = edge.second;

        if (visited.find(dest) != visited.end()) {
            continue; // Skip if already visited
        }

        mST.push_back({src, dest});
        totalCost += cost;

        visited.insert(dest);

        // Add adjacent edges of the destination vertex to the priority queue
        for (auto adjEdge : getEdges(dest)) {
            unsigned adjDest = adjEdge->getDest()->getId();
            if (visited.find(adjDest) == visited.end()) {
                pq.push({adjEdge->getDistance(), {dest, adjDest}});
            }
        }

        // Also consider all non-adjacent vertices for potential edges
        for (auto vertex : vertexSet) {
            unsigned vertexId = vertex->getId();
            if (visited.find(vertexId) == visited.end()) {
                double dist = getDistanceOrHaversine(dest, vertexId);
                if (dist != INF) {
                    pq.push({dist, {dest, vertexId}});
                }
            }
        }
    }

    return totalCost;
}



Edge* Graph::getEdge(int sourceId, int destId) const {
    // First, find the source vertex
    Vertex* sourceVertex = findVertex(sourceId);
    if (!sourceVertex) {
        std::cerr << "Source vertex with ID " << sourceId << " not found." << std::endl;
        return nullptr;
    }

    // Now, find the edge with the destination vertex ID
    for (Edge* edge : sourceVertex->getAdj()) {
        if (edge->getDest()->getId() == destId) {
            return edge;
        }
    }

    // If no edge found, print an error message and return nullptr
    std::cerr << "Edge from vertex " << sourceId << " to vertex " << destId << " not found." << std::endl;
    return nullptr;
}

double Graph::getDistance(int sourceId, int destId) const {
    Vertex* sourceVertex = findVertex(sourceId);
    Vertex* destVertex = findVertex(destId);
    if (sourceVertex && destVertex) {
        Edge* edge = sourceVertex->getEdge(destId);
        if (edge) {
            return edge->getDistance();
        }
    }
    return INF; // If there's no edge, return infinity or some large value
}

Vertex* Graph::getVertex(unsigned id) const {
    for (auto vertex : vertexSet) {
        if (vertex->getId() == id) {
            return vertex;
        }
    }
    return nullptr;
}


// Function to calculate Haversine distance between two latitude-longitude points
double Graph::haversine(double lat1, double lon1, double lat2, double lon2) const {
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


Graph Graph::createToyGraphs(const std::string& graphFile) {
    Graph graph; // Create a new graph instance to populate

    // Add ".csv" extension to the provided file name
    std::string filePath = "../Toy-Graphs/" + graphFile;

    std::ifstream infile(filePath);
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        exit(1); // Or handle the error more gracefully
    }

    std::string line;
    bool firstLine = true; // Skip header line if exists

    while (std::getline(infile, line)) {
        if (firstLine) {
            // Skip the first line (considered as a header)
            firstLine = false;
            continue;
        }

        std::istringstream iss(line);
        int source = 0, dest = 0;
        double distance;
        char comma;
        // Read source, destination, and distance
        if (!(iss >> source >> comma >> dest >> comma >> distance)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue; // Skip invalid lines
        }

        // Add or find vertices and add edges
        Vertex* sourceVertex = graph.findVertex(source);
        if (!sourceVertex) {
            sourceVertex = new Vertex(source);
            graph.addVertex(*sourceVertex);
        }
        Vertex* destVertex = graph.findVertex(dest);
        if (!destVertex) {
            destVertex = new Vertex(dest);
            graph.addVertex(*destVertex);
        }

        // Add the edge
        sourceVertex->addEdge(destVertex, distance);
        // Add the reverse edge
        destVertex->addEdge(sourceVertex, distance);
    }
    infile.close();

    return graph;
}


Graph Graph::createExtraFullyConnectedGraph(const std::string& graphFile) {
    Graph graph; // Create a new graph instance to populate

    std::string filePath = "../Extra_Fully_Connected_Graphs/" + graphFile;

    std::ifstream infile(filePath);
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        exit(1);
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int source = 0, dest = 0;
        double distance;
        char comma;
        if (!(iss >> source >> comma >> dest >> comma >> distance)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        }

        Vertex *sourceVertex = graph.findVertex(source);
        if (!sourceVertex) {
            sourceVertex = new Vertex(source);
            graph.addVertex(*sourceVertex);
        }
        Vertex *destVertex = graph.findVertex(dest);
        if (!destVertex) {
            destVertex = new Vertex(dest);
            graph.addVertex(*destVertex);
        }

        sourceVertex->addEdge(destVertex, distance);
        destVertex->addEdge(sourceVertex, distance);
    }
    infile.close();

    return graph;
}

Graph Graph::createRealWorldGraphs(const std::string& graphFile) {
    Graph graph; // Create a new graph instance to populate

    std::ifstream infile(graphFile);
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << graphFile << std::endl;
        exit(1);
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int source = 0, dest = 0;
        double distance;
        char comma;
        if (!(iss >> source >> comma >> dest >> comma >> distance)) {
            continue;
        }

        Vertex *sourceVertex = graph.findVertex(source);
        if (!sourceVertex) {
            sourceVertex = new Vertex(source);
            graph.addVertex(*sourceVertex);
        }
        Vertex *destVertex = graph.findVertex(dest);
        if (!destVertex) {
            destVertex = new Vertex(dest);
            graph.addVertex(*destVertex);
        }

        sourceVertex->addEdge(destVertex, distance);
        destVertex->addEdge(sourceVertex, distance);
    }
    infile.close();

    return graph;
}


void Graph::printGraph(const Graph& graph) {
    std::cout << "Graph:" << std::endl;
    for (const auto& vertex : graph.getVertices()) {
        std::cout << "Vertex " << vertex->getId() << ": ";
        for (const auto& edge : vertex->getAdj()) {
            std::cout << edge->getDest()->getId() << " ";
        }
        std::cout << std::endl;
    }
}

Edge* Graph::findEdge(unsigned first, unsigned second) const{
    // Get the adjacency list for the vertex with id 'first'
    for (auto edge : vertexSet[first]->getAdj()) {
        // Check if the edge connects 'first' to 'second'
        if (edge->getOrig()->getId() == second || edge->getDest()->getId() == second) {
            return edge;
        }
    }
    return nullptr;
}

double Graph::tSP2OptImprovement(std::vector<int>& path) {
    double currDistance = 0;
    for (int i = 0; i < path.size() - 1; i++)
        currDistance += findEdge(path[i], path[i + 1])->getDistance();

    double bestDistance = currDistance;

    bool found = true;
    while (found) {
        found = false;

        for (int i = 1; i < path.size() - 2; i++) {
            for (int j = i + 1; j < path.size() - 1; j++) {

                std::vector<int> newPath = path;

                currDistance -= findEdge(newPath[i - 1], newPath[i])->getDistance();
                currDistance -= findEdge(newPath[j], newPath[j + 1])->getDistance();

                if (j != i + 1) { // Non-consecutive nodes
                    currDistance -= findEdge(newPath[i], newPath[i + 1])->getDistance();
                    currDistance -= findEdge(newPath[j - 1], newPath[j])->getDistance();
                }

                std::swap(newPath[i], newPath[j]);

                currDistance += findEdge(newPath[i - 1], newPath[i])->getDistance();
                currDistance += findEdge(newPath[j], newPath[j + 1])->getDistance();

                if (j != i + 1) { // Non-consecutive nodes
                    currDistance += findEdge(newPath[i], newPath[i + 1])->getDistance();
                    currDistance += findEdge(newPath[j - 1], newPath[j])->getDistance();
                }

                if (currDistance < bestDistance) {
                    path = newPath;
                    bestDistance = currDistance;
                    found = true;
                    break;
                }
                else currDistance = bestDistance;
            }
        }
        if (found) break;
    }
    return bestDistance;
}



void Graph::dfsTree(unsigned startId, std::vector<unsigned>& tree) const {
    // Clear the previous content of the tree vector
    tree.clear();

    // Create a set to keep track of visited vertices
    std::unordered_set<unsigned> visited;

    // Perform DFS traversal
    dfsHelper(startId, visited, tree);
}


void Graph::dfsHelper(unsigned currentId, std::unordered_set<unsigned>& visited, std::vector<unsigned>& tree) const {
    // Mark the current vertex as visited
    visited.insert(currentId);

    // Add the current vertex to the tree
    tree.push_back(currentId);

    // Find the current vertex in the vector
    Vertex* currentVertex = nullptr;
    for (auto vertex : vertexSet) {
        if (vertex->getId() == currentId) {
            currentVertex = vertex;
            break;
        }
    }

    if (currentVertex) {
        // Iterate over adjacent vertices
        for (const auto& edge : currentVertex->getAdj()) {
            unsigned adjId = edge->getDest()->getId();
            // If the adjacent vertex is not visited, recursively call DFS on it
            if (visited.find(adjId) == visited.end()) {
                dfsHelper(adjId, visited, tree);
            }
        }
    }
}

