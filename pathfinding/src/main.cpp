/**
 * \file 	main.cpp
 * \author	Quentin Chateau
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \brief 	Main file of the pathfinding project.
 * \date  	03/04/2017
 * \copyright Copyright (c) 2017 UTCoupe All rights reserved.
 *
 * The pathfinding program is communicating using string commands.
 * The protocol is defined as follow : cmd;x1;y1;x2;y2;...\n
 * Where cmd can be :
 *  C : compute a path (args are x_start:y_start;x_end;y_end)
 *  D : refresh the internal dynamic objects (args are x;y;radius)
 * Except the cmd, all the other values are integers.
 * The pathfinding answers a computing command with the valid path found,
 * with respect of the format : x_start;y_start;x1;y1;...;x_end;y_end;path_length\n
 * In this case, all values are integers, except the path_length which is float.
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <tclap/CmdLine.h>
#include "lib/map.hpp"

#define FAILED_STR "FAIL\n"

using namespace std;
using namespace TCLAP;

// Declaration of command line options
/**
 * Flag to activate the debug mode (add more verbose)
 */
bool debugFlag = false;
/**
 * Flag to activate the bmp rendering
 */
bool bmpRenderingFlag = false;
/**
 * Relative or absolute path to the map to use (bmp format)
 */
string mapPath = "";
/**
 * Heuristic mode to use to compute distance
 */
heuristic_type heuristicMode = NORM1;

/**
 * A dynamic object is a moving robot.
 * The parameters are the cartesian position (in mm) and the radius of the objectf (in mm).
 */
typedef struct dynamic_object {
    unsigned int x, y, r;
} dynamic_object;

/**
 * Parse the command line options and sets the corresponding global variables.
 * @param argc The number of command line arguments
 * @param argv The array of command line arguments
 */
void parseOptions(int argc, char **argv);

/**
 * Checks if the parameter point is in the map
 * @param x X coordinate (in pixel)
 * @param y Y coordinate (in pixel)
 * @param map The map object to use as base
 * @return True if the coordinate is valid, otherwise false
 */
inline bool isValid(unsigned int x, unsigned int y, MAP &map) {
    return x >= 0 && x < map.get_map_w() && y >= 0 && y < map.get_map_h();
}

/**
 * Adds dynamic objects in the map
 * @param command The string containing the command of the objects to add (cmd;x1;y1;x2;y2;...)
 * @param map The map object where to add the dynamic objects
 */
void addDynamicObject(string &command, MAP &map) {
    vector<dynamic_object> objs;
    // Remove the "cmd;"
    command = command.substr(2);
    if (debugFlag) {
        cout << "Objects this time :" << endl;
    }
    // Finds and adds all the objects in command
    while (command.find(';') != string::npos) {
        dynamic_object obj;
        sscanf(command.c_str(), "%i;%i;%i", &obj.x, &obj.y, &obj.r);
        objs.push_back(obj);
        command = command.substr(command.find(';') + 1);
        command = command.substr(command.find(';') + 1);
        command = command.substr(command.find(';') + 1);
        if (debugFlag) {
            cout << obj.x << ":" << obj.y << ":" << obj.r << endl;
        }
    }
    // Clear the dynamic barriers because there are new objects
    map.clear_dynamic_barriers();
    for (auto &obj: objs) {
        map.add_dynamic_circle(obj.x, obj.y, obj.r);
    }
}

/**
 * Compute a path from start point to end point
 * If start point is not valid, find the nearest valid point
 * if end point is not valid, do the same, except if the end point is in a dynamic barrier (aka a robot) then return \n immediatly
 * @param command The string containing : x_start;y_start;x_end;y_end\n
 * @param map The map object used to compute the path
 * @return If valid, returns the path as a string (x_start;y_start;x1;y1;...;xn;xy;x_end;y_end;path_length\n) with integers values except the path_length (float)
 *         If not valid, returns FAILED value
 */
string commandCalcPath(string &command, MAP &map) {
    unsigned int x_s, y_s, x_e, y_e;
    vertex_descriptor start, end, start_valid, end_valid;
    vector<vertex_descriptor> path;
    double distance;
    stringstream answer;
    chrono::time_point<chrono::system_clock> startChrono, endChrono;
    chrono::duration<double> elapsedSeconds;

    // Get the initial and end point and search the nearest valid point

    command = command.substr(2);
    if (sscanf(command.c_str(), "%i;%i;%i;%i", &x_s, &y_s, &x_e, &y_e) < 4) {
        cerr << "Did not parse the input correctly" << endl;
        return FAILED_STR;
    }
    if (!(isValid(x_s, y_s, map) && isValid(x_e, y_e, map))) {
        cerr << "Start or end point is not in the map" << endl;
        return FAILED_STR;
    }

    end = map.get_vertex(x_e, y_e);
    if (map.has_dynamic_barrier(end)) {
        cerr << "End point on a dynamic object" << endl;
        return FAILED_STR;
    }
    end_valid = map.find_nearest_valid(end);

    start = map.get_vertex(x_s, y_s);
    start_valid = map.find_nearest_valid(start);

    if (debugFlag) {
        cout << "Start : " << start_valid[0] << ":" << start_valid[1] << endl;
        cout << "End : " << end_valid[0] << ":" << end_valid[1] << endl;
        startChrono = chrono::system_clock::now();
    }

    // Ask the map to compute the path between the start and the end
    map.solve(start_valid, end_valid);
    if (!map.solved()) {
        cerr << "Could not find any path" << endl;
        return FAILED_STR;
    }
    // Transform the path to be smoother
    map.solve_smooth();
    // Get the result path
    path = map.get_smooth_solution();
    distance = map.get_smooth_solution_length();
    // Check if the path is valid
    if (start != start_valid) {
        path.insert(path.begin(), start);
        distance += heuristicCompute(heuristicMode, start).computeHeuristic(start_valid);
    }
    if (end != end_valid) {
        path.push_back(end);
        distance += heuristicCompute(heuristicMode, end).computeHeuristic(end_valid);
    }
    // Create the string representing the path
    for (auto &point: path) {
        answer << point[0] << ";" << point[1] << ";";
    }
    answer << distance << endl;
    if (debugFlag) {
        endChrono = chrono::system_clock::now();
        elapsedSeconds = endChrono - startChrono;
        cout << "Path contains " << path.size() << " points, total distance = " << distance << endl;
        cout << "Computing time : " << elapsedSeconds.count() << endl;
    }
    if (bmpRenderingFlag) {
        map.generate_bmp("tmp.bmp");
    }
    return answer.str();
}

int main(int argc, char **argv) {
    // Parse the command line options
    parseOptions(argc, argv);

    if (debugFlag) {
        cout << "Loading map " << mapPath << endl;
        switch (heuristicMode) {
            case EUCLIDEAN:
                cout << "Using euclidean heuristic" << endl;
                break;
            case NORM1:
                cout << "Using norm1 heuristic" << endl;
                break;
        }
    }
    MAP map(mapPath);
    map.set_heuristic_mode(heuristicMode);
    if (debugFlag) {
        cout << "Done, map size is : " << map.get_map_w()
             << "x" << map.get_map_h() << endl;
    }
    while (std::cin.good()) {
        string command, answer;
        cin >> command;
        switch (command[0]) {
            case 'D':
                addDynamicObject(command, map);
                break;
            case 'C':
                answer = commandCalcPath(command, map);
                cout << answer;
                cout.flush();
                break;
            default:
                cerr << "Default : " << command[0] << endl;
                break;
        }
    }
    cout << "Communication with system failed, stop the pathfinding" << endl;
}

void parseOptions(int argc, char **argv) {
    try {
        CmdLine cmd("Command description message", ' ', "0.1");

        ValueArg<string> mapArg("m", "map", "Path to the map used to compute pathfinding.", true, "", "string");
        cmd.add(mapArg);

        ValueArg<uint8_t> heuristicArg("e", "heuristic",
                                       "Heuristic mode for pathfinding computing (EUCLIDIEAN = 0, NORM1 = 1).", false,
                                       1, "uint8_t");
        cmd.add(heuristicArg);

        SwitchArg debugArg("d", "debug", "Set the debug flag.", cmd, false);

        SwitchArg renderingArg("r", "rendering", "Set the rendering flag.", cmd, false);

        cmd.parse(argc, argv);

        mapPath = mapArg.getValue();
        heuristicMode = (heuristic_type) heuristicArg.getValue();
        debugFlag = debugArg.getValue();
        bmpRenderingFlag = renderingArg.getValue();
    } catch (ArgException &e) { // catch any exceptions
        cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
    }
}
