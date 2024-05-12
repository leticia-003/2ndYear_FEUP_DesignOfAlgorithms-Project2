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


// Compare structure for priority queue
struct Compare {
    bool operator()(std::pair<double, Vertex*> const& p1, std::pair<double, Vertex*> const& p2) {
        return p1.first > p2.first; // Min-heap based on edge distance
    }
};


double Graph::mstPrim(int startId, std::vector<std::pair<unsigned, unsigned>>& mST) const {
    std::priority_queue<std::pair<double, std::pair<unsigned, unsigned>>, std::vector<std::pair<double, std::pair<unsigned, unsigned>>>, std::greater<std::pair<double, std::pair<unsigned, unsigned>>>> pq;
    std::unordered_set<unsigned> visited;
    mST.clear();

    // Push the start vertex and its edges into the priority queue
    for (auto edge : getEdges(startId)) {
        pq.push({edge->getDistance(), {startId, edge->getDest()->getId()}});
    }
    visited.insert(startId);

    double totalCost = 0.0;

    while (!pq.empty()) {
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

Graph* Graph::buildRealWorldGraph(unsigned number) {
    auto graph = new Graph;
    buildRealWorldGraphNodes(number, graph);
    buildRealWorldGraphEdges(number, graph);
    return graph;
}

void Graph::buildRealWorldGraphNodes(unsigned number, Graph* graph) {
    std::ifstream file("../Real-world Graphs/graph" + std::to_string(number) + "/nodes.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening nodes file for graph " << number << std::endl;
        return;
    }

    std::string line;
    getline(file, line); // Skip header

    while (getline(file, line)) {
        std::string id, latitude, longitude;
        std::stringstream input(line);
        getline(input, id, ',');
        getline(input, longitude, ',');
        getline(input, latitude, '\r');

        // Create a new vertex
        Vertex* newVertex = new Vertex(std::stoi(id));

        // Add node to the graph
        if (!graph->addVertex(*newVertex)) {
            std::cerr << "Error adding vertex with ID " << id << " to the graph (already exists)" << std::endl;
            delete newVertex; // Clean up if adding fails
        }
    }
    file.close();
}



void Graph::buildRealWorldGraphEdges(unsigned number, Graph* graph) {
    std::ifstream file("../Real-world Graphs/graph" + std::to_string(number) + "/edges.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening edges file for graph " << number << std::endl;
        return;
    }

    std::string line;
    getline(file, line); // Skip header

    while (getline(file, line)) {
        std::string first, second, distance;
        std::stringstream input(line);
        getline(input, first, ',');
        getline(input, second, ',');
        getline(input, distance, '\r');

        // Find the vertices in the graph
        Vertex* firstVertex = graph->findVertex(std::stoi(first));
        Vertex* secondVertex = graph->findVertex(std::stoi(second));

        // If vertices are found, add the edge
        if (firstVertex && secondVertex) {
            firstVertex->addEdge(secondVertex, std::stod(distance));
        } else {
            std::cerr << "Error: Vertices not found for edge: " << first << " - " << second << std::endl;
        }
    }
    file.close();
}


void Graph::printGraph(const Graph* graph) {
    std::cout << "Graph:" << std::endl;
    for (const auto& vertex : graph->getVertices()) {
        std::cout << "Vertex " << vertex->getId() << ": ";
        for (const auto& edge : vertex->getAdj()) {
            std::cout << edge->getDest()->getId() << " ";
        }
        std::cout << std::endl;
    }
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

