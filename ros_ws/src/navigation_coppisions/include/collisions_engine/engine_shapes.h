#ifndef ENGINE_SHAPES_H
#define ENGINE_SHAPES_H

#include "engine_shapes_attrib.h"
#include <vector>


class MapObstacle
{
public:
    ObstacleType type = ObstacleType::OBSTACLE;
    Position position;
    Velocity velocity;
    float spawn_time;

    MapObstacle();
    MapObstacle(Point point, Velocity velocity = nullptr);
    MapObstacle(Position position, Velocity velocity = nullptr);
    ~MapObstacle();
};


class SegmentObstacle: public MapObstacle
{
public:
    Point first, last;
    float length;

    SegmentObstacle(Point first_point, Point last_point, Velocity velocity = nullptr);
    ~SegmentObstacle();
};


class CircleObstacle: public MapObstacle
{
public:
    float radius;

    CircleObstacle(Position position, float radius, Velocity velocity = nullptr);
    ~CircleObstacle();
};

class RectObstacle: public MapObstacle
{
public:
    float width, height;

    RectObstacle(Position position, float width, float height, Velocity velocity = nullptr);
    std::vector<Point> get_corners();
    std::vector<SegmentObstacle> get_segments();
    ~RectObstacle();
};

#endif