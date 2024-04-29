#include "Menu.h"
#include "Graphs.h"
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

        std::vector<std::vector<Vertex*>> toyGraphs = graphHandler.createToyGraphs(graphFiles[graphChoice - 1]);
        // Let's say you want to print the first toy graph
        graphHandler.printGraph(toyGraphs[0]); // Passing the first toy graph to printGraph
    }

    return dataset;
}
