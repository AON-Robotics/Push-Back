#include <iostream>
#include "RDP.hpp"

int main() {
    aon::RDP rdp;

    std::vector<aon::RDP::Point> path = {
        {0,0},
        {1,0},
        {2,1},
        {3,0},
        {4,0}
    };
    for (auto i : path) {
        std::cout << "{" << i.x << ", " << i.y << "}\n";
    }
    std::cout << "\nSimplify:\n";

    rdp.setELIPSON(100);
    std::vector<aon::RDP::Point> simplify = rdp.simplify(path);

    for (auto i : simplify) {
        std::cout << "{" << i.x << ", " << i.y << "}\n";
    }

    return 0;
}
