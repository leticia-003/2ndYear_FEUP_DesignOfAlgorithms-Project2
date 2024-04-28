#include "Menu.h"
#include <iostream>

void displayMenu() {
    std::cout << "Select a dataset to analyze:" << std::endl;
    std::cout << "1. Toy-Graphs" << std::endl;
    std::cout << "2. Extra Fully Connected Graphs" << std::endl;
    std::cout << "3. Real-World Graphs" << std::endl;
}

std::string getDatasetChoice() {
    int choice;
    std::string dataset;

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

    return dataset;
}
