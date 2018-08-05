#ifndef ENGINE_H
#define ENGINE_H

#include "engine_constants.h"
#include "engine_shapes.h"
#include <vector>

class Collision
{
public:
    CollisionLevel level;
    MapObstacle obstacle;
    float approx_distance;

    Collision(CollisionLevel level, MapObstacle obstacle, float approx_distance);
    ~Collision();
};


class CollisionsResolver
{
public:
    static std::vector<MapObstacle> find_collisions(std::vector<MapObstacle> robot_shapes, 
                                                    std::vector<MapObstacle> obstacles_shapes);
    static bool intersect(MapObstacle obs1, MapObstacle obs2);

private:
    CollisionsResolver();
    ~CollisionsResolver();

    bool _segments_intersect(SegmentObstacle segment1, SegmentObstacle segment2);
    bool _circles_intersect(CircleObstacle circle1, CircleObstacle circle2);
    bool _rects_intersect(RectObstacle rect1, RectObstacle rect2);
    bool _segment_intersects_rect(SegmentObstacle segment, RectObstacle rect);
    bool _segment_intersects_circle(SegmentObstacle segment, CircleObstacle circle);
    bool _point_in_rect(Point point, RectObstacle rect);
    bool _point_in_circle(Point point, CircleObstacle circle);
    bool _rect_intersects_circle(RectObstacle rect, CircleObstacle circle);
};

#endif