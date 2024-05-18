//
// Created by Letícia Coelho on 03/05/2024.
//

#ifndef DA_PROJECT2_ALGORITHMS_H
#define DA_PROJECT2_ALGORITHMS_H
#include <iostream>
#include <vector>
#include "VertexEdge.h"
using namespace std;

/**
 * @class Algorithms
 * @brief A class implementing various algorithms for solving graph problems, particularly focusing on the Traveling Salesman Problem (TSP).
 */
class Algorithms {
public:
    /**
     * @brief Constructs an Algorithms object with a given graph.
     * @param g Pointer to the graph.
     */
    Algorithms(const Graph* g) : graph(g) {}

    /**
     * @brief Executes the backtracking algorithm for the TSP on the given graph and saves the results to a file.
     * @param graph The graph on which to perform the TSP.
     * @param graphFile The file to save the TSP path.
     * @complexity O(n!), where n is the number of vertices in the graph.
     */
    void backtrackingAlgorithm(const Graph& graph, const std::string& graphFile) const;

    /**
     * @brief Calculates the distance between two vertices in the graph.
     * @param graph The graph containing the vertices.
     * @param sourceId The ID of the source vertex.
     * @param destId The ID of the destination vertex.
     * @return The distance between the source and destination vertices.
     * @complexity O(1)
     */
    static double getDistance(const Graph& graph, int sourceId, int destId);

    /**
     * @brief Solves the TSP using a backtracking approach.
     * @param graph The graph on which to perform the TSP.
     * @param currentVertex The current vertex in the path.
     * @param count The count of vertices visited so far.
     * @param currentPath The current path taken.
     * @param visited The boolean array representing visited vertices.
     * @param currentCost The current cost of the path.
     * @param minCost The minimum cost found.
     * @param bestPath The best path found.
     * @return The minimum cost of the TSP path.
     * @complexity O(n!), where n is the number of vertices in the graph.
     */
    double tspBacktracking(const Graph& graph, unsigned currentVertex, unsigned count, std::vector<unsigned>& currentPath, std::vector<bool>& visited, double currentCost, double& minCost, std::vector<unsigned>& bestPath) const;

    /**
     * @brief Approximates the solution to the TSP using a 2-approximation algorithm starting from a given vertex.
     * @param startId The starting vertex ID.
     * @param tspPath The path found by the algorithm.
     * @return The approximate cost of the TSP path.
     * @complexity O(n^2), where n is the number of vertices in the graph.
     */
    double tsp2Approximation(int startId, std::vector<int>& tspPath) const;

    /**
     * @brief Performs a preorder traversal on a Minimum Spanning Tree (MST) adjacency list to find a TSP path.
     * @param currentId The current vertex ID in the traversal.
     * @param mstAdjList The adjacency list of the MST.
     * @param visited The set of visited vertices.
     * @param path The path found by the traversal.
     * @complexity O(n), where n is the number of vertices in the graph.
     */
    void preorderTraversal(int currentId, const std::unordered_map<int, std::vector<int>>& mstAdjList, std::unordered_set<int>& visited, std::vector<int>& path) const;

    /**
     * @brief Performs a depth-first search (DFS) traversal to find a TSP path.
     * @param u The current vertex.
     * @param parent The parent vertex.
     * @param adjList The adjacency list of the graph.
     * @param visited The boolean array representing visited vertices.
     * @param tspPath The path found by the traversal.
     * @complexity O(n), where n is the number of vertices in the graph.
     */
    void dfsTraversal(int u, int parent, const std::vector<std::vector<int>>& adjList,
                      std::vector<bool>& visited, std::vector<int>& tspPath) const;

    /**
     * @brief Solves the TSP using the nearest neighbor heuristic.
     * @param graph The graph on which to perform the TSP.
     * @param startNode The starting vertex ID.
     * @return A pair containing the cost of the path and the path itself.
     * @complexity O(n^2), where n is the number of vertices in the graph.
     */
    static std::pair<double, std::vector<int>> nearestNeighbor(const Graph& graph, int startNode);

    /**
     * @brief Improves the given TSP path using the 2-opt local search method.
     * @param graph The graph on which to perform the TSP.
     * @param path The TSP path to be improved.
     * @return The improved cost of the TSP path.
     * @complexity O(n^2), where n is the number of vertices in the graph.
     */
    static double tSP2OptImprovement(const Graph& graph, std::vector<int>& path);

    /**
     * @brief Prints the TSP tour.
     * @param tour The TSP path to be printed.
     * @complexity O(n), where n is the number of vertices in the path.
     */
    static void printTour(const std::vector<int>& tour);

private:
    const Graph* graph; ///< Pointer to the graph.
};

#endif //DA_PROJECT2_ALGORITHMS_H
