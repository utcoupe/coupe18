/**
 * \file 	map.cpp
 * \author	Quentin Chateau
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \brief 	This file defines the map and its relatives objects
 * \date  	03/04/2017
 * \copyright Copyright (c) 2017 UTCoupe All rights reserved.
 */

#include "pathfinding/map.hpp"

using namespace std;

MAP::MAP(const std::string &map_filename) :
        image(map_filename) {

    map_h = image.height();
    map_w = image.width();

    map = new grid(create_map((size_t)map_w, (size_t)map_h));
    map_barrier = new filtered_grid(create_barrier_map());

    for (unsigned int y = 0; y < map_h; y++) {
        for (unsigned int x = 0; x < map_w; x++) {
            unsigned char r, g, b;
            image.get_pixel(x, y, r, g, b);
            if (b < 127) {
                vertex_descriptor u = get_vertex(x, y);
                static_barriers.insert(u);
            }
        }
    }

    barriers = static_barriers;
}

MAP::~MAP() {
    delete map;
    delete map_barrier;
}

void MAP::add_dynamic_circle(unsigned int x, unsigned int y, unsigned int f_r) {
//    unsigned int r = (unsigned int)ceil(f_r);
    int r = (int)f_r;
    int r2 = r*r;
    for (int p_x = (int)x - r; p_x <= (int)x + r; p_x++) {
        if (p_x < 0 || p_x >= (int)map_w) {
            continue;
        }

        // Caution with std pow ! 2 standard function exist but be careful of particularities around float zeros !!
        // We chose to use standard int multiplication instead
        int y_length = (int)ceil(sqrt((float)r2 - (float)(((int)x - p_x)*((int)x - p_x))));
        for (int p_y = (int)y - y_length; p_y <= (int)y + y_length; p_y++) {
            if (p_y < 0 || p_y >= (int)map_h) {
                // cout<<"Escape p_y = "<<p_y<<endl;
                continue;
            }
            vertex_descriptor u = get_vertex(p_x, p_y);
            if (!has_dynamic_barrier(u)) {
                dynamic_barriers.insert(u);
                if (!has_barrier(u)) {
                    barriers.insert(u);
                }
            }
        }
    }
    clear_solution();
}

void MAP::clear_dynamic_barriers() {
    dynamic_barriers.clear();
    barriers = static_barriers;
    clear_solution();
}

bool MAP::solve(vertex_descriptor source, vertex_descriptor dest) {
    clear_solution();
    boost::static_property_map<double> weight(1);
    // The predecessor map is a vertex-to-vertex mapping.
    typedef boost::unordered_map<vertex_descriptor,
            vertex_descriptor,
            vertex_hash> pred_map;
    pred_map predecessor;
    boost::associative_property_map<pred_map> pred_pmap(predecessor);
    // The distance map is a vertex-to-distance mapping.
    typedef boost::unordered_map<vertex_descriptor,
            double,
            vertex_hash> dist_map;
    dist_map distance;
    boost::associative_property_map<dist_map> dist_pmap(distance);

    v_start = source;
    v_end = dest;

    astar_goal_visitor visitor(v_end);

    try {
        heuristicCompute heuristic(h_mode, v_end);
        astar_search(*map_barrier, v_start, heuristic,
                     boost::weight_map(weight).
                             predecessor_map(pred_pmap).
                             distance_map(dist_pmap).
                             visitor(visitor));
    } catch (found_goal fg) {
        // Walk backwards from the goal through the predecessor chain adding
        // vertices to the solution path.
        for (vertex_descriptor u = v_end; u != v_start; u = predecessor[u])
            solution.push_back(u);
        solution.push_back(v_start);
        solution_length = distance[v_end];
        return true;
    }
    return false;
}

void MAP::solve_smooth() {
    if (!solved()) return;
    double distance;
    vertex_descriptor last = v_start;
    smooth_solution_length = 0;
    smooth_solution.push_back(v_start);
    // solution.begin is the last vertex in the path
    do {
        for (auto it = solution.begin(); *it != last; ++it) {
            if (get_direct_distance(last, *it, distance)) {
                smooth_solution.push_back(*it);
                smooth_solution_length += distance;
                last = *it;
                break;
            }
        }
    } while (last != v_end);
}

vertex_descriptor MAP::find_nearest_valid(vertex_descriptor u) {
    int dist = 0;
    vertex_descriptor nearest = u;
    vector<vertex_descriptor> v_this_dist;
    while (has_barrier(nearest)) {
        if (v_this_dist.size() == 0) {
            ++dist;
            for (int x = (int)((unsigned int) u[0] - dist); x <= (int)((unsigned int) u[0] + dist); ++x) {
                if (x < 0 || x > (int)map_w) continue;
                for (int y = (int)((unsigned int) u[1] - dist); y <= (int)((unsigned int) u[1] + dist); ++y) {
                    if (y < 0 || y > (int)map_h) continue;
                    vertex_descriptor v = get_vertex(x, y);
                    if (heuristicCompute(h_mode, u).computeHeuristic(v) == dist) {
                        v_this_dist.push_back(v);
                    }
                }
            }
        }
        //todo return bool to know if a nearest has been found ?
        nearest = v_this_dist.back();
        v_this_dist.pop_back();
    }
    return nearest;
}

void MAP::generate_bmp(string path) {
    bitmap_image img(map_w, map_h);
    for (unsigned int y = 0; y < map_h; y++) {
        for (unsigned int x = 0; x < map_w; x++) {
            vertex_descriptor u = {{(vertices_size_type) x, (vertices_size_type) y}};
            if (solution_contains(u))
                img.set_pixel(x, y, 0, 255, 0);
            else if (has_barrier(u))
                img.set_pixel(x, y, 0, 0, 0);
            else
                img.set_pixel(x, y, 255, 255, 255);
        }
    }
    image_drawer draw(img);
    draw.pen_width(3);
    draw.pen_color(50, 50, 255);
    for (auto &p: smooth_solution) {
        draw.plot_pen_pixel((int) p[0], (int) p[1]);
    }
    draw.pen_width(1);
    draw.pen_color(255, 0, 0);
    for (unsigned int i = 1; i < smooth_solution.size(); i++) {
        int x1, x2, y1, y2;
        x1 = (int) smooth_solution[i - 1][0];
        x2 = (int) smooth_solution[i][0];
        y1 = (int) (smooth_solution[i - 1][1]);
        y2 = (int) (smooth_solution[i][1]);
        draw.line_segment(x1, y1, x2, y2);
    }
    img.save_image(path);
}

/* 			*
 *  PRIVATE *
 * 			*/

bool MAP::get_direct_distance(vertex_descriptor &v, vertex_descriptor &goal, double &d) {
    double m, c;
    long dx, dy;
    int inc;
    dx = (long) goal[0] - (long) v[0];
    dy = (long) goal[1] - (long) v[1];
    if (abs(dx) > abs(dy)) {
        // iterate over X
        if (dx > 0) {
            inc = 1;
        } else {
            inc = -1;
        }
        m = (double) dy / dx;
        c = v[1] - m * v[0];
        for (int x = (int) v[0]; x != (long) goal[0]; x += inc) {
            int y = (int) round(m * x + c);
            if (has_barrier(get_vertex(x, y))) {
                d = 0;
                return false;
            }
        }
    } else {
        // iterate over Y
        if (dy > 0) {
            inc = 1;
        } else {
            inc = -1;
        }
        m = (double) dx / dy;
        c = v[0] - m * v[1];
        for (int y = (int) v[1]; y != (long) goal[1]; y += inc) {
            int x = (int) round(m * y + c);
            if (has_barrier(get_vertex(x, y))) {
                d = 0;
                return false;
            }
        }
    }
    d = heuristicCompute(h_mode, goal).computeHeuristic(v);
    return true;
}

vertex_descriptor MAP::get_vertex(int x, int y) {
    return vertex(x + (y * map_w), *map);
}

grid MAP::create_map(std::size_t x, std::size_t y) {
    boost::array<std::size_t, 2> lengths = {{x, y}};
    return grid(lengths);
}

filtered_grid MAP::create_barrier_map() {
    return boost::make_vertex_subset_complement_filter(*map, barriers);
}

#define BARRIER "#"

// Print the maze as an ASCII map.
std::ostream& MAP::operator<<(std::ostream &output) {
    output << "Grid size : " << get_map_w() << "x" << get_map_h() << std::endl;
    // Header
    for (unsigned int i = 0; i < get_map_w() + 2; i++)
        output << BARRIER;
    output << std::endl;
    // Body
    for (unsigned int y = 0; y < get_map_h(); y++) {
        // Put a barrier on the left-hand side.
        output << BARRIER;
        for (unsigned int x = 0; x < get_map_w(); x++) {
            // Put the character representing this point in the maze grid.
            vertex_descriptor u = {{(vertices_size_type) x, (vertices_size_type) y}};
            if (smooth_solution_contains(u))
                output << "O";
            else if (solution_contains(u))
                output << ".";
            else if (has_barrier(u))
                output << BARRIER;
            else
                output << " ";
        }
        // Put a barrier on the right-hand side.
        output << BARRIER;
        // Put a newline after every row except the last one.
        output << std::endl;
    }
    // Footer
    for (unsigned int i = -1; i < get_map_w() + 2; i++)
        output << BARRIER;
    return output;
}
