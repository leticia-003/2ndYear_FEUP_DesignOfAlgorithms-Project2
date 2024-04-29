#include <iostream>
#include "Menu.h"

int main() {
    std::string dataset = getDatasetChoice();
    std::cout << "You chose to analyze: " << dataset << std::endl;

    return 0;
}
