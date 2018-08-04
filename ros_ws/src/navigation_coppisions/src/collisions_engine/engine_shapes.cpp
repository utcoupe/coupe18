#include "engine_shapes.h"
#include <math.h>

MapObstacle::MapObstacle() //TODO good c++ ..?
{
    MapObstacle::position = Position(0, 0, 0);
    MapObstacle::velocity = Velocity(0, 0);
    float spawn_time = 0;
}
MapObstacle::MapObstacle(Point point, Velocity velocity = nullptr)
{
    MapObstacle::position = Position(point.x, point.y, 0);
    MapObstacle::velocity = velocity;
    float spawn_time = 0;
}
MapObstacle::MapObstacle(Position position, Velocity velocity = nullptr)
{
    MapObstacle::position = position;
    MapObstacle::velocity = velocity;
    float spawn_time = 0;
}


SegmentObstacle::SegmentObstacle(Point first_point, Point last_point, Velocity velocity = nullptr)
    :MapObstacle(first_point, velocity)
{
    ObstacleType type = ObstacleType::SEGMENT;
    first = first_point;
    last  = last_point;

    length = sqrt(pow(last.x - first.x, 2) + pow(last.y - last.y, 2));
    position = Position((first.x + last.x) / 2, (first.y + last.y) / 2, 
                        atan2(last.y - first.y, last.x - first.x)); // center position
}


CircleObstacle::CircleObstacle(Position position, float radius, Velocity velocity = nullptr)
    :MapObstacle(position, velocity)
{
    ObstacleType type = ObstacleType::CIRCLE;
    CircleObstacle::radius = radius;
}


RectObstacle::RectObstacle(Position position, float width, float height, Velocity velocity = nullptr)
    :MapObstacle(position, velocity)
{
    ObstacleType type = ObstacleType::RECT;
    RectObstacle::width = width;
    RectObstacle::height = height;
}

std::vector<Point> RectObstacle::get_corners()
{
    std::vector<Point> corners;
    float l = sqrt(pow(width / 2.0, 2) + pow(height / 2.0, 2));
    float corner_phi = atan2(height, width);

    float angle_phi = 0;
    for(int j = 0; j < 2; j++) {
        for(int i = 0; i < 2; i++)
        {
            float phi = angle_phi + pow(i + 1, -1) * corner_phi;
            corners.push_back(Point(position.x + l * cos(phi + position.a), 
                                    position.y + l * sin(phi + position.a)));
        }
        angle_phi += M_PI;
    }
    return corners;
}

std::vector<SegmentObstacle> RectObstacle::get_segments()
{
    std::vector<Point> corners = get_corners();

    std::vector<SegmentObstacle> segments;
    for(int i = 0; i < 4; i++)
        segments.push_back(SegmentObstacle(corners[i], corners[(i + 1) % 4]));
    
    return segments;
}