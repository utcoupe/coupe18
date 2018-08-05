#ifndef ENGINE_SHAPES_H
#define ENGINE_SHAPES_H

#include "engine_shapes_attrib.h"
#include <vector>
#include "stdint.h"


class MapObstacle
{
public:
    ObstacleType type = ObstacleType::OBSTACLE;
    Position position;
    Velocity velocity;
    uint32_t spawn_time;

    MapObstacle();
    MapObstacle(Point point, Velocity velocity = Velocity());
    MapObstacle(Position position, Velocity velocity = Velocity());
    ~MapObstacle();
};


class SegmentObstacle: public MapObstacle
{
public:
    Point first, last;
    float length;

    SegmentObstacle(Point first_point, Point last_point, Velocity velocity = Velocity());
    ~SegmentObstacle();
};


class CircleObstacle: public MapObstacle
{
public:
    float radius;

    CircleObstacle(Position position, float radius, Velocity velocity = Velocity());
    ~CircleObstacle();
};

class RectObstacle: public MapObstacle
{
public:
    float width, height;

    RectObstacle(Position position, float width, float height, Velocity velocity = Velocity());
    std::vector<Point> get_corners();
    std::vector<SegmentObstacle> get_segments();
    ~RectObstacle();
};

#endif