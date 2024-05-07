#include "Menu.h"
#include "Graphs.h"
#include "Algorithms.h"
#include <iostream>
#include <filesystem>

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
        std::cout << "Enter your choice (1, 2, or 3): ";
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
        Algorithms algorithms;

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

        std::cout << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;
        std::cout << "|          Choose an action           |" << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;
        std::cout << "|  1. Print the graph                 |" << std::endl;
        std::cout << "|  2. TSP Backtracking                |" << std::endl;
        std::cout << "+-------------------------------------+" << std::endl;

        int actionChoice;
        std::cout << "Enter your choice (1 or 2): ";
        std::cin >> actionChoice;

        double minCost = 0.0;
        double elapsedTime;
        std::vector<int> bestPath;

        switch (actionChoice) {
            case 1:
                // Print the graph
                graphHandler.printGraph(toyGraph);
                break;
            case 2:
                // Perform TSP backtracking
                algorithms.backtrackingAlgorithm(toyGraph, graphFiles[graphChoice - 1]);
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

        std::vector<std::vector<Vertex*>> fullyConnectedGraphs = graphHandler.createExtraFullyConnectedGraphs(filePath);

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

        std::string folderName;
        if (graphChoice == 1) {
            folderName = "graph1";
        } else if (graphChoice == 2) {
            folderName = "graph2";
        } else if (graphChoice == 3) {
            folderName = "graph3";
        }

        std::string filePath = "../Real-World Graphs/" + folderName + "/edges.csv";

        std::vector<std::vector<Vertex*>> realWorldGraphs = graphHandler.createRealWorldGraphs(filePath);

    }

    return dataset;
}
