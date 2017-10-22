# Pathfinding node

## How to use
### Current version
Usage :
```
rosrun pathfinding pathfinding -m %s [-d] [-h %d] [-r]
```
with the following arguments:
* `-m %s` (required): specify the file where the map is stored (suported format: `.bmp`)
* `-d` (optional): launch the program in debug mode. It will try to set its loglevel in ROS to debug.
* `-e %d` (optional): set the pathfinding heuristic mode between `0` (euclidean) and `1` (norm1).
* `-r` (optional): the program will output an image (`tmp.bmp`) showing results after computing

It will provide two servers:
* `/navigation/pathfinding/findpath`: See `srv/FindPath.srv`
* `/navigation/pathfinding/doorder`: Receive an order in string format. All parameters have to be seperated by a `;`. The first is a letter (ASCII) :
- `C` followed by 4 parameters `x_s`, `y_s`, `x_e`, `y_e` which are the start and end positions to join. It will return a string with a list of point folowed by the total distance in it.
- `D` followed by n*3 parameters. Each group of parameters are `x`, `y`, `r` representing a circle barrier. It will not respond anything.

## How it works
### Current version
The program opens an image containing the static barriers of the game. Black pixels represent forbiden access and white ones clear access. This image is converted to a `boost::grid_graph<2>` and the position are exprimed with `boost::vertex`.

The algorithm is A*. The program use the `boost::astar_search` with `boost::default_astar_visitor`. It creates a grid and mark the vertex (`boost::graph_traits<boost::grid_graph<2>>::vertex_descriptor`) when it "visits" it. Because it starts on the start position and travel in the graph step by step by priotizing the smallest distances, at the end it will found the shortest path between the start position and the end position. All vertexes have the same weigh in the graph.

The visitor can go relatively to its position to `(0,1)`, `(0,-1)`, `(1,0)`, `(-1,0)`, if they are valid. When he finds the end position, it writes the path and raises an exception (here it's `found_goal`). Else it returns.

With the shortest path, the program then smoothes it by removing unnecessary waypoints (if we have `a` then `b` then `c` and we can draw a line between `a` and `c` without meeting any barriers then we can delete `b`).

For another example of `boost::astar_search`, look there : https://github.com/wpm/Astar-Maze-Solver