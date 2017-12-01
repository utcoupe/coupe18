#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <chrono>
#include <sstream>
#include <utility>
#include <vector>

#include <ros/console.h>

#include "navigation_pathfinder/FindPath.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/point.h"
#include "pathfinder/pos_convertor.h"

class Pathfinder
{
public:
    typedef std::vector<Point> Path;

    Pathfinder(const std::string& mapFileName, const std::pair< double, double >& tableSize, bool invertedY = true, bool render = false);
    
    bool findPath(const Point& startPos, const Point& endPos, Path& path);
    bool findPathCallback(navigation_pathfinder::FindPath::Request &req, navigation_pathfinder::FindPath::Response &rep);

private:
    typedef std::vector<std::vector<short> > Vect2DShort;
    typedef std::vector<std::vector<bool> > Vect2DBool;
    
    PosConvertor _convertor;
    
    MapStorage _mapStorage;
    
    Vect2DBool _allowedPositions;
    Vect2DBool _dynBarrierPositions;
    
    bool _renderAfterComputing;
    
    bool exploreGraph(Vect2DShort& distMap, const Point& startPos, const Point& endPos);
    Path retrievePath(const Vect2DShort& distMap, const Point& startPos, const Point& endPos);
    Path smoothPath(const Path& rawPath);
    
    bool isValid(const Point& pos);
    bool canConnectWithLine(const Point& pA, const Point& pB);
    
    std::vector<Point> directions() const;
    
    // Convertors
    Point pose2DToPoint(const geometry_msgs::Pose2D& pos) const;
    geometry_msgs::Pose2D pointToPose2D(const Point& pos) const;
    
    std::string pathMapToStr(const Path& path);
    std::string pathRosToStr(const std::vector<geometry_msgs::Pose2D>& path);
};

#endif // PATHFINDER_H
