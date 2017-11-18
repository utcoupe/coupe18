#ifndef SHARED_DECLARATIONS_H
#define SHARED_DECLARATIONS_H

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

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

typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;
typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type filtered_grid;

struct found_goal {
};

typedef enum heuristic_type {
    EUCLIDEAN, NORM1
} heuristic_type;

struct astar_goal_visitor : public boost::default_astar_visitor {
    astar_goal_visitor(vertex_descriptor goal) : m_goal(goal) {};

    void examine_vertex(vertex_descriptor u, const filtered_grid &) {
        if (u == m_goal)
            throw found_goal();
    }

private:
    vertex_descriptor m_goal;
};

#endif // SHARED_DECLARATIONS_H
