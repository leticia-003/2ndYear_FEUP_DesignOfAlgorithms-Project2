#include "Graphs.h"
#include <cmath>
#include <fstream>
#include <queue>
#include <unordered_set>
#include <sstream>
using namespace std;

Graph::~Graph() {
    for (auto& pair : vertexMap) {
        delete pair.second; // Delete each vertex
    }
}

const std::unordered_map<int, Vertex *> & Graph::getVertexMap() const {
    return vertexMap;
}

unsigned Graph::size() const {
    return vertexMap.size();
}

Vertex * Graph::findVertex(const int &id) const {
    auto iter = vertexMap.find(id);
    if (iter != vertexMap.end()) {
        return iter->second;
    } else {
        return nullptr;
    }
}

bool Graph::addVertex(Vertex &vertex) {
    if (findVertex(vertex.getId()) != nullptr)  // This check might be redundant due to how vertices are added
        return false;
    vertexMap.insert({vertex.getId(), &vertex});  // Insert the pointer to the existing vertex
    return true;
}

std::vector<Vertex*> Graph::getVertices() const {
    std::vector<Vertex*> vertices;
    for (const auto& pair : vertexMap) {
        vertices.push_back(pair.second);
    }
    return vertices;
}

// Compare structure for priority queue
struct Compare {
    bool operator()(std::pair<double, Vertex*> const& p1, std::pair<double, Vertex*> const& p2) {
        return p1.first > p2.first; // Min-heap based on edge distance
    }
};

double Graph::mstPrim(int startId) {
    // Get all vertices in the graph
    std::vector<Vertex*> vertices = getVertices();
    if (vertices.empty()) {
        std::cerr << "The graph is empty." << std::endl;
        return 0.0;
    }

    // Find the starting vertex
    Vertex* startVertex = nullptr;
    for (Vertex* vertex : vertices) {
        if (vertex->getId() == startId) {
            startVertex = vertex;
            break;
        }
    }

    if (!startVertex) {
        std::cerr << "Starting vertex not found." << std::endl;
        return 0.0;
    }

    // Priority queue to select the edge with the smallest weight
    std::priority_queue<std::pair<double, Vertex*>, std::vector<std::pair<double, Vertex*>>, Compare> pq;

    // Set to keep track of vertices included in MST
    std::unordered_set<int> inMST;

    // Total cost of the MST
    double totalCost = 0.0;

    // Add the starting vertex to the priority queue with 0 distance
    pq.push({0.0, startVertex});

    while (!pq.empty()) {
        // Get the vertex with the smallest edge weight
        auto [currentDist, currentVertex] = pq.top();
        pq.pop();

        // Check if the vertex is already included in MST
        if (inMST.find(currentVertex->getId()) != inMST.end()) {
            continue;
        }

        // Include this vertex in MST and add the edge cost
        inMST.insert(currentVertex->getId());
        totalCost += currentDist;

        // Process all adjacent vertices
        for (Edge* edge : currentVertex->getAdj()) {
            Vertex* adjVertex = edge->getDest();
            if (inMST.find(adjVertex->getId()) == inMST.end()) {
                pq.push({edge->getDistance(), adjVertex});
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

bool Graph::isComplete() const {
    // Get the total number of vertices in the graph
    unsigned n = vertexMap.size();

    // Calculate the expected number of edges in a complete graph with 'n' vertices
    unsigned expectedEdges = (n * (n - 1)) / 2;

    // Count the actual number of edges in the graph
    unsigned actualEdges = 0;

    // Iterate over each vertex in the graph
    for (const auto& vertexPair : vertexMap) {
        Vertex* vertex = vertexPair.second;

        // Count the number of edges connected to this vertex
        actualEdges += vertex->getAdj().size();
    }

    // Check if the actual number of edges matches the expected number
    return actualEdges == expectedEdges;
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
            std::cout << "Created new source vertex: " << source << std::endl;
        }
        Vertex *destVertex = graph.findVertex(dest);
        if (!destVertex) {
            destVertex = new Vertex(dest);
            graph.addVertex(*destVertex);
            std::cout << "Created new destination vertex: " << dest << std::endl;
        }

        std::cout << "Adding edge from " << source << " to " << dest << " with distance " << distance << std::endl;
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


