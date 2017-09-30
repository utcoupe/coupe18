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
#include <cmath>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "bitmap_image.hpp"

class MAP;

typedef boost::grid_graph<2> grid;
typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;

struct vertex_hash : std::unary_function<vertex_descriptor, std::size_t> {
    std::size_t operator()(vertex_descriptor const &u) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, u[0]);
        boost::hash_combine(seed, u[1]);
        return seed;
    }
};

struct found_goal {
};

typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;
typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type filtered_grid;

typedef enum heuristic_type {
    EUCLIDEAN, NORM1
} heuristic_type;

std::ostream &operator<<(std::ostream &os, const MAP &map);

class heuristicCompute : public boost::astar_heuristic<filtered_grid, double> {
public:
    heuristicCompute(heuristic_type type, vertex_descriptor goal) : m_goal(goal), m_type(type) {};

    double computeHeuristic(vertex_descriptor v) {
        double returnValue = 0.0;
        if (m_type == NORM1) {
            returnValue = std::abs((double)m_goal[0] - (double)v[0]) + std::abs((double)m_goal[1] - (double)v[1]);
        } else if (m_type == EUCLIDEAN) {
            returnValue = sqrt(pow(double(m_goal[0]) - double(v[0]), 2) + pow(double(m_goal[1]) - double(v[1]), 2));
        }
        return returnValue;
    }

private:
    vertex_descriptor m_goal;
    heuristic_type m_type;
};


struct astar_goal_visitor : public boost::default_astar_visitor {
    astar_goal_visitor(vertex_descriptor goal) : m_goal(goal) {};

    void examine_vertex(vertex_descriptor u, const filtered_grid &) {
        if (u == m_goal)
            throw found_goal();
    }

private:
    vertex_descriptor m_goal;
};

/**
 * The map object computes paths based on an internal map.
 * The inputs of it is a bitmap file defining the area where to compute path (called static map).
 * In this picture, all black pixels are the forbidden areas and the white ones are allowed areas.
 * Based on this static map, some dynamic objects can be added. Each dynamic object defines an other forbidden area.
 * Then, with this dynamic map, the map object tries to compute a valid path between a start and end point.
 */
class MAP {
public:
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
