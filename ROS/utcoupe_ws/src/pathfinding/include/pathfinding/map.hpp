/**
 * \file 	map.hpp
 * \author	Quentin Chateau
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \brief 	This file defines the map and its relatives objects
 * \date  	03/04/2017
 * \copyright Copyright (c) 2017 UTCoupe All rights reserved.
 */

#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>

#include "bitmap_image.hpp"
#include "shared_declarations.hpp"
#include "heuristic_compute.hpp"

/**
 * The map object computes paths based on an internal map.
 * The inputs of it is a bitmap file defining the area where to compute path (called static map).
 * In this picture, all black pixels are the forbidden areas and the white ones are allowed areas.
 * Based on this static map, some dynamic objects can be added. Each dynamic object defines an other forbidden area.
 * Then, with this dynamic map, the map object tries to compute a valid path between a start and end point.
 */
class MAP {
public:
    MAP() {}
    MAP(const std::string &map_filename);
    ~MAP();

    unsigned int get_map_h() const { return map_h; };
    unsigned int get_map_w() const { return map_w; };
    void add_dynamic_circle(unsigned int x, unsigned int y, unsigned int f_r);
    void clear_dynamic_barriers();

    bool solve(vertex_descriptor source, vertex_descriptor dest);

    bool solve(int x_source, int y_source,
               int x_dest, int y_dest) {
        return solve(get_vertex(x_source, y_source),
                     get_vertex(x_dest, y_dest));
    }

    void solve_smooth();

    bool solution_contains(vertex_descriptor u) const {
        for (const auto &el: solution) {
            if (el == u)
                return true;
        }
        return false;
    }

    bool smooth_solution_contains(vertex_descriptor u) const {
        for (const auto &el: smooth_solution) {
            if (el == u)
                return true;
        }
        return false;
    }

    bool has_barrier(vertex_descriptor u) const {
        return (barriers.find(u) != barriers.end());
    }

    bool has_dynamic_barrier(vertex_descriptor u) const {
        return (dynamic_barriers.find(u) != dynamic_barriers.end());
    }

    void generate_bmp(std::string path);
    vertex_descriptor find_nearest_valid(vertex_descriptor u);

    vertex_descriptor get_vertex(int x, int y);

    double get_smooth_solution_length() { return smooth_solution_length; };

    std::vector<vertex_descriptor> get_smooth_solution() { return smooth_solution; };

    bool solved() const { return !solution.empty(); }

    void set_heuristic_mode(heuristic_type mode) { h_mode = mode; };
    
    /**
     * Print the maze as an ASCII map.
     */
    std::ostream &operator<<(std::ostream &output);
    
private:
    unsigned int map_w, map_h;
    grid create_map(std::size_t x, std::size_t y);
    filtered_grid create_barrier_map();
    bool get_direct_distance(vertex_descriptor &v, vertex_descriptor &goal,
                             double &d);

    void clear_solution() {
        solution.clear();
        solution_length = 0;
        smooth_solution.clear();
    }

    bitmap_image image;
    grid *map;
    filtered_grid *map_barrier;
    vertex_set barriers, static_barriers, dynamic_barriers;
    std::vector<vertex_descriptor> solution, smooth_solution;
    vertex_descriptor v_start, v_end;
    heuristic_type h_mode;
    double solution_length, smooth_solution_length;
};

#endif
