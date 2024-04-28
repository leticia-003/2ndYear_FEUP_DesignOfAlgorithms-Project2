#include <iostream>
#include "Menu.h"

int main() {
    std::string dataset = getDatasetChoice();
    std::cout << "You chose to analyze: " << dataset << std::endl;
    // Add further processing or function calls based on the selected dataset

    return 0;
}
