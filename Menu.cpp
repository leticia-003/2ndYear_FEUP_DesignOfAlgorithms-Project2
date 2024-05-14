#include "Menu.h"
#include "Graphs.h"
#include "Algorithms.h"
#include <iostream>
#include <filesystem>
#include <cmath>

void displayMenu() {
    std::cout << "+-------------------------------------+" << std::endl;
    std::cout << "|          Select a Dataset           |" << std::endl;
    std::cout << "+-------------------------------------+" << std::endl;
    std::cout << "|  1. Toy-Graphs                      |" << std::endl;
    std::cout << "|  2. Extra Fully Connected Graphs    |" << std::endl;
    std::cout << "|  3. Real-World Graphs               |" << std::endl;
    std::cout << "+-------------------------------------+" << std::endl;
    std::cout << "Enter your choice (1, 2, or 3): ";
}

std::string getDatasetChoice() {
    int choice;
    std::string dataset;
    Graph graphHandler;

    while (true) {
        displayMenu();
        std::cin >> choice;

        switch (choice) {
            case 1:
                dataset = "Toy-Graphs";
                break;
            case 2:
                dataset = "Extra Fully Connected Graphs";
                break;
            case 3:
                dataset = "Real-World Graphs";
                break;
            default:
                std::cout << "Invalid choice. Please enter a number between 1 and 3." << std::endl;
                continue;
        }

        break;
    }

    if (dataset == "Toy-Graphs") {

        // List files in the Toy-Graphs directory
        std::vector<std::string> graphFiles;
        std::cout << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;
        std::cout << "|         Available Toy-Graphs        |" << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;
        std::cout << "|  1. Tourism                         |" << std::endl;
        std::cout << "|  2. Stadiums                        |" << std::endl;
        std::cout << "|  3. Shipping                        |" << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;


        int fileNum = 1;
        for (const auto& entry : std::__fs::filesystem::directory_iterator("../Toy-Graphs")) {
            std::string filename = entry.path().filename();
            graphFiles.push_back(filename);
            ++fileNum;
        }

        // Ask the user which graph to load
        int graphChoice;
        do {
            std::cout << "Enter your choice (1, 2, or 3): ";
            std::cin >> graphChoice;
        } while (graphChoice < 1 || graphChoice > graphFiles.size());

        Graph toyGraph = graphHandler.createToyGraphs(graphFiles[graphChoice - 1]);

        Algorithms algo(&toyGraph);

        std::cout << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;
        std::cout << "|          Choose an action              |" << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;
        std::cout << "|  1. Print the graph                    |" << std::endl;
        std::cout << "|  2. TSP Backtracking                   |" << std::endl;
        std::cout << "|  3. MST Prim                           |" << std::endl;
        std::cout << "|  4. Triangular Approximation Heuristic |" << std::endl;
        std::cout << "|  5. Nearest Neighbor Heuristic         |" << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;

        int actionChoice;
        std::cout << "Enter your choice (1, 2, 3, 4 or 5): ";
        std::cin >> actionChoice;

        std::vector<int> bestPath;

        switch (actionChoice) {
            case 1:
                // Print the graph
                //graphHandler.printGraph(toyGraph);
                break;
            case 2:
                // Perform TSP backtracking
                algo.backtrackingAlgorithm(toyGraph, graphFiles[graphChoice - 1]);
                break;
            case 3: {
                auto startTime = std::chrono::high_resolution_clock::now();
                int startId = 0; // Starting vertex ID is fixed at 0

                // Call mstPrim function
                std::vector<std::pair<unsigned, unsigned>> mST;
                double mstCost = toyGraph.mstPrim(startId, mST);

                // Output the cost of the Minimum Spanning Tree
                std::cout << "The cost of the Minimum Spanning Tree is: " << mstCost << std::endl;

                auto endTime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
                std::cout << "Time taken: " << duration << " ms" << std::endl;
                break;
            }
            case 4:
                if (graphChoice == 3) {
                    std::cout << "The graph is not fully connected. Unable to apply the Triangular Approximation Heuristic." << std::endl;
                } else {
                    auto startTime = std::chrono::high_resolution_clock::now();
                    int startId = 0;  // Or whichever vertex you want to start from
                    std::vector<int> tspPath;
                    double tspCost = algo.tsp2Approximation(startId, tspPath);

                    std::cout << "TSP Path: ";
                    for (size_t i = 0; i < tspPath.size(); ++i) {
                        std::cout << tspPath[i];
                        if (i < tspPath.size() - 1) {
                            std::cout << " -> ";
                        }
                    }
                    std::cout << std::endl;

                    std::cout << "Total Cost of TSP Path: " << tspCost << std::endl;
                    auto endTime = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
                    std::cout << "Time taken: " << duration << " ms" << std::endl;
                }
                break;

            case 5:

                if (graphChoice == 3) {
                    std::cout << "The graph is not fully connected. Unable to apply the Triangular Approximation Heuristic." << std::endl;
                } else {
                    int startNode = 0;
                    auto nnResult = algo.nearestNeighbor(toyGraph, startNode);
                    double nnCost = nnResult.first;
                    std::vector<int> nnTour = nnResult.second;
                    std::cout << "Nearest Neighbor Tour Cost: " << nnCost << std::endl;
                    std::cout << "Nearest Neighbor Tour: ";
                    algo.printTour(nnTour);

                    // Improve the tour using Simulated Annealing
                    /*auto improvedResult = algo.simulatedAnnealing(toyGraph, nnTour);
                    double improvedCost = improvedResult.first;
                    std::vector<int> improvedTour = improvedResult.second;
                    std::cout << "Improved Tour Cost (Simulated Annealing): " << improvedCost << std::endl;
                    std::cout << "Improved Tour (Simulated Annealing): ";
                    algo.printTour(improvedTour);*/

                    double resultado = toyGraph.tSP2OptImprovement(nnTour);
                    cout << resultado;

                }
            break;

            default:
                std::cout << "Invalid choice" << std::endl;
        }
    }

    if (dataset == "Extra Fully Connected Graphs") {

        std::vector<std::string> graphFiles = {
                "edges_25.csv",
                "edges_50.csv",
                "edges_75.csv",
                "edges_100.csv",
                "edges_200.csv",
                "edges_300.csv",
                "edges_400.csv",
                "edges_500.csv",
                "edges_600.csv",
                "edges_700.csv",
                "edges_800.csv",
                "edges_900.csv"
        };

        std::cout << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;
        std::cout << "|     Choose the number of NODES      |" << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;
        std::cout << "|  1.  25 Nodes                       |" << std::endl;
        std::cout << "|  2.  50 Nodes                       |" << std::endl;
        std::cout << "|  3.  75 Nodes                       |" << std::endl;
        std::cout << "|  4.  100 Nodes                      |" << std::endl;
        std::cout << "|  5.  200 Nodes                      |" << std::endl;
        std::cout << "|  6.  300 Nodes                      |" << std::endl;
        std::cout << "|  7.  400 Nodes                      |" << std::endl;
        std::cout << "|  8.  500 Nodes                      |" << std::endl;
        std::cout << "|  9.  600 Nodes                      |" << std::endl;
        std::cout << "|  10. 700 Nodes                      |" << std::endl;
        std::cout << "|  11. 800 Nodes                      |" << std::endl;
        std::cout << "|  12. 900 Nodes                      |" << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;

        int graphChoice;
        do {
            std::cout << "Enter your choice (1-12): ";
            std::cin >> graphChoice;
        } while (graphChoice < 1 || graphChoice > 12);

        std::string filePath = "../Extra_Fully_Connected_Graphs/" + graphFiles[graphChoice - 1];

        Graph fullyConnectedGraphs = graphHandler.createExtraFullyConnectedGraph(filePath);

        std::cout << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;
        std::cout << "|          Choose an action              |" << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;
        std::cout << "|  1. Print the graph                    |" << std::endl;
        std::cout << "|  2. MST Prim                           |" << std::endl;
        std::cout << "|  3. Triangular Approximation Heuristic |" << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;

        int actionChoice;
        std::cout << "Enter your choice (1, 2 or 3): ";
        std::cin >> actionChoice;

        std::vector<int> bestPath;

        switch (actionChoice) {
            case 1:
                // Print the graph
                graphHandler.printGraph(fullyConnectedGraphs);
                break;
            case 2: {
                auto startTime = std::chrono::high_resolution_clock::now();
                int startId = 0; // Starting vertex ID is fixed at 0

                // Call mstPrim function
                std::vector<std::pair<unsigned, unsigned>> mST;
                double mstCost = fullyConnectedGraphs.mstPrim(startId, mST);

                // Output the cost of the Minimum Spanning Tree
                std::cout << "The cost of the Minimum Spanning Tree is: " << mstCost << std::endl;

                auto endTime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
                std::cout << "Time taken: " << duration << " ms" << std::endl;
            }
            break;
            case 3: {
                auto startTime = std::chrono::high_resolution_clock::now();
                int startId = 0;  // Or whichever vertex you want to start from
                std::vector<int> tspPath;
                Algorithms algo(&fullyConnectedGraphs);
                double tspCost = algo.tsp2Approximation(startId, tspPath);

                std::cout << "TSP Path: ";
                for (size_t i = 0; i < tspPath.size(); ++i) {
                    std::cout << tspPath[i];
                    if (i < tspPath.size() - 1) {
                        std::cout << " -> ";
                    }
                }
                std::cout << std::endl;

                if(tspCost>1000000){
                    double arredondado = round(tspCost / 1000000.0 * 100.0) / 100.0; // Arredonda para duas casas decimais
                    std::cout << "Total Cost of TSP Path: " << arredondado << "M" << std::endl;
                }
                else {
                    std::cout << "Total Cost of TSP Path: " << tspCost << std::endl;
                }

                auto endTime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
                std::cout << "Time taken: " << duration << " ms" << std::endl;
            }
            break;

            default:
                std::cout << "Invalid choice" << std::endl;
        }

    }

    if (dataset == "Real-World Graphs") {

        std::cout << std::endl;
        std::cout << "+-----------------------------------------+" << std::endl;
        std::cout << "|       Available Real World Graphs       |" << std::endl;
        std::cout << "+-----------------------------------------+" << std::endl;
        std::cout << "|  1. Graph 1 (1K nodes and ~500K edges)  |" << std::endl;
        std::cout << "|  2. Graph 2 (5K nodes and ~4M edges)    |" << std::endl;
        std::cout << "|  3. Graph 3 (10K nodes and ~10M edges)  |" << std::endl;
        std::cout << "+-----------------------------------------+" << std::endl;

        int graphChoice;
        do {
            std::cout << "Enter your choice (1, 2, or 3): ";
            std::cin >> graphChoice;
        } while (graphChoice < 1 || graphChoice > 3);

        std::string graphDirectory;
        switch (graphChoice) {
            case 1:
                graphDirectory = "graph1";
                break;
            case 2:
                graphDirectory = "graph2";
                break;
            case 3:
                graphDirectory = "graph3";
                break;
        }



        std::string filePath = "../Real-world Graphs/" + graphDirectory + "/edges.csv";

        Graph realWorldGraph = graphHandler.createRealWorldGraphs(filePath);

        graphHandler.parseNodesFile(graphDirectory, realWorldGraph);

        std::cout << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;
        std::cout << "|          Choose an action              |" << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;
        std::cout << "|  1. Print the graph                    |" << std::endl;
        std::cout << "|  2. MST                                |" << std::endl;
        std::cout << "|  3. Triangular Approximation Heuristic |" << std::endl;
        std::cout << "+----------------------------------------+" << std::endl;

        int actionChoice;
        std::cout << "Enter your choice (1, 2 or 3): ";
        std::cin >> actionChoice;

        switch (actionChoice) {
            case 1:
                // Print the graph
                graphHandler.printGraph(realWorldGraph);
                break;
            case 2: {
                auto startTime = std::chrono::high_resolution_clock::now();
                int startId = 0; // Starting vertex ID is fixed at 0

                // Call mstPrim function
                std::vector<std::pair<unsigned, unsigned>> mST;
                double mstCost = realWorldGraph.mstPrim(startId, mST);

                // Output the cost of the Minimum Spanning Tree
                std::cout << "The cost of the Minimum Spanning Tree is: " << mstCost << std::endl;

                auto endTime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
                std::cout << "Time taken: " << duration << " ms" << std::endl;
            }
            break;

            case 3: {
                auto startTime = std::chrono::high_resolution_clock::now();
                int startId = 0;  // Or whichever vertex you want to start from
                std::vector<int> tspPath;
                Algorithms algo(&realWorldGraph);
                double tspCost = algo.tsp2Approximation(startId, tspPath);

                std::cout << "TSP Path: ";
                for (size_t i = 0; i < tspPath.size(); ++i) {
                    std::cout << tspPath[i];
                    if (i < tspPath.size() - 1) {
                        std::cout << " -> ";
                    }
                }
                std::cout << std::endl;

                if(tspCost>1000000){
                    double arredondado = round(tspCost / 1000000.0 * 100.0) / 100.0; // Arredonda para duas casas decimais
                    std::cout << "Total Cost of TSP Path: " << arredondado << "M" << std::endl;
                }
                else {
                    std::cout << "Total Cost of TSP Path: " << tspCost << std::endl;
                }

                auto endTime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
                std::cout << "Time taken: " << duration << " ms" << std::endl;
            }
                break;
            default:
                std::cout << "Invalid choice" << std::endl;
        }


    }

    return dataset;
}
