#ifndef HEURISTIC_COMPUTE_H
#define HEURISTIC_COMPUTE_H

#include <cmath>

#include "shared_declarations.hpp"

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

#endif // HEURISTIC_COMPUTE_H
