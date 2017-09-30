/**
 * \file 	map_test.cpp
 * \author	Quentin Chateau
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \brief 	Main file for the program to test the pathfinding
 * \date  	03/04/2017
 * \copyright Copyright (c) 2017 UTCoupe All rights reserved.
 *
 * This program is kind of interactive.
 * When launched, you have to enter the path of the bmp map file to use (with relative or absolute path).
 * Then you have to enter the points defining the start point and the end point :
 * x_start (in pixel)
 * y_start (in pixel)
 * x_end (in pixel)
 * y_end (in pixel)
 * The test program will return the found path solution (if there is one) and the computing time.
 */

#include <iostream>
#include <time.h>
#include "../lib/map.hpp"

int main() {
    std::string path;
    std::cout << "Path to bmp (relative or absolute) :" << std::endl;
    std::cin >> path;
    MAP map(path);
    std::cout << "Map is " << map.get_map_w() << "x" << map.get_map_h() << std::endl;
    std::cout << "Select source and destination :" << std::endl;
    clock_t teu, tn1, t = clock();
    unsigned int x_s, y_s, x_e, y_e;
    std::cout << "x_start (in pixel) :" << std::endl;
    std::cin >> x_s;
    std::cout << "y_start (in pixel) :" << std::endl;
    std::cin >> y_s;
    std::cout << "x_end (in pixel) :" << std::endl;
    std::cin >> x_e;
    std::cout << "y_end (in pixel) :" << std::endl;
    std::cin >> y_e;

    // EUCLIDEAN
    map.set_heuristic_mode(EUCLIDEAN);
    map.solve(x_s, y_s, x_e, y_e);
    map.solve_smooth();
    teu = clock() - t;
    std::cout << std::endl << "EUCLIDEAN" << std::endl;
    std::cout << "Smooth solution length : " << map.get_smooth_solution_length() << std::endl;
    printf("Time : %f seconds.\n", ((float) teu) / CLOCKS_PER_SEC);
    map.generate_bmp(path + "-euclidean.bmp");
    if (map.get_map_w() < 80 && map.get_map_h() < 50)
        std::cout << map << std::endl;

    // NORM1
    t = clock();
    map.set_heuristic_mode(NORM1);
    map.solve(x_s, y_s, x_e, y_e);
    map.solve_smooth();
    tn1 = clock() - t;
    std::cout << std::endl << "NORM1" << std::endl;
    std::cout << "Smooth solution length : " << map.get_smooth_solution_length() << std::endl;
    printf("Time : %f seconds.\n", ((float) tn1) / CLOCKS_PER_SEC);
    map.generate_bmp(path + "-norm1.bmp");
    if (map.get_map_w() < 80 && map.get_map_h() < 50)
        std::cout << map << std::endl;

    return 0;
}
