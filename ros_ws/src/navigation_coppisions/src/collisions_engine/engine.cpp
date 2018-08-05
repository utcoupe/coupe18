#include "engine.h"
#include <math>

Collision::Collision(CollisionLevel level, MapObstacle obstacle, float approx_distance)
{
    Collision::level = level;
    Collision::obstacle = obstacle;
    Collision::approx_distance = approx_distance;
}


std::vector<MapObstacle> CollisionsResolver::find_collisions(std::vector<MapObstacle> robot_shapes, 
                                                             std::vector<MapObstacle> obstacles_shapes)
{
    std::vector<MapObstacle> collisions;
    
    for(int rs_i = 0; rs_i < robot_shapes.size(); rs_i++)
    {
        for(int os_i = 0; os_i < robot_shapes.size(); os_i++)
        {
            bool intersecting = false;
            if (intersect(robot_shapes[rs_i], obstacles_shapes[os_i]))
            { // check for shape-to-shape collisions
                collisions.push_back(obstacles_shapes[os_i]);
                intersecting = true;
            }

            if (obstacles_shapes[os_i].velocity != nullptr && !intersecting)
            { // if the obstacle has a velocity, check its velocity zone too
                VelocityCheckZone _check_zone = new VelocityCheckZone(obstacles_shapes[os_i].velocity.width, 
                                                                      obstacles_shapes[os_i].velocity.height, 
                                                                      CollisionLevel::LEVEL_STOP);
                std::vector<MapObstacle> vel_shapes = _check_zone.get_shapes(obstacles_shapes[os_i].position);
                for(int vs_i = 0; vs_i < vel_shapes.size(); vs_i++)
                {
                    if(intersect(robot_shapes[rs_i], vel_shapes[vs_i]))
                        collisions.push_back(vel_shapes[vs_i]);
                }
            }
        }
    }
    
    return collisions;
}

bool CollisionsResolver::intersect(MapObstacle obs1, MapObstacle obs2)
{//dynamic_cast<Tile*>(o)
    ObstacleType t1 = obs1.type;
    ObstacleType t2 = obs2.type;

    if(t1 == ObstacleType::RECT and t2 == ObstacleType::RECT)
        return _rects_intersect(obs1, obs2);
    else if(t1 == ObstacleType::CIRCLE and t2 == ObstacleType::CIRCLE)
        return _circles_intersect(obs1, obs2);
    else if(t1 == ObstacleType::SEGMENT and t2 == ObstacleType::SEGMENT)
        return _segments_intersect(obs1, obs2);
    else if(t1 == ObstacleType::POINT and t2 == ObstacleType::POINT)
        return false;
    else if((t1 == ObstacleType::RECT || t2 == ObstacleType::RECT) && (t1 == ObstacleType::CIRCLE || t2 == ObstacleType::CIRCLE))
    {
        if(t1 == ObstacleType::RECT)
            return _rect_intersects_circle(obs1, obs2);
        return _rect_intersects_circle(obs2, obs1);
    }
    else if((t1 == ObstacleType::SEGMENT || t2 == ObstacleType::SEGMENT) && (t1 == ObstacleType::RECT || t2 == ObstacleType::RECT))
    {
        if(t1 == ObstacleType::SEGMENT)
            return _segment_intersects_rect(obs1, obs2);
        return _segment_intersects_rect(obs2, obs1);
    }
    else if((t1 == ObstacleType::SEGMENT || t2 == ObstacleType::SEGMENT) && (t1 == ObstacleType::CIRCLE || t2 == ObstacleType::CIRCLE))
    {
        if(t1 == ObstacleType::SEGMENT)
            return _segment_intersects_circle(obs1, obs2);
        return _segment_intersects_circle(obs2, obs1);
    }
    else if((t1 == ObstacleType::RECT || t2 == ObstacleType::RECT) && (t1 == ObstacleType::POINT || t2 == ObstacleType::POINT))
    {
        if(t1 == ObstacleType::RECT)
            return _point_in_rect(obs1, obs2);
        return _rect_intersects_circle(obs2, obs1);
    }
    else if((t1 == ObstacleType::CIRCLE || t2 == ObstacleType::CIRCLE) && (t1 == ObstacleType::POINT || t2 == ObstacleType::POINT))
    {
        if(t1 == ObstacleType::POINT)
            return _point_in_rect(obs1, obs2);
        return _rect_intersects_circle(obs2, obs1);
    }
    ROS_LOGERR("Couldn't compare intersection between objects.");
    return false;
}

bool CollisionsResolver::_segments_intersect(SegmentObstacle segment1, SegmentObstacle segment2)
{
    std::vector<Point> s1 = {segment1.first, segment1.last};
    std::vector<Point> s2 = {segment2.first, segment2.last};

    // https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
    auto ccw = [](Point a, Point b, Point c) { 
        return (c.y-a.y) * (b.x-a.x) > (b.y-a.y) * (c.x-a.x); 
    };
    return ccw(s1[0],s2[0],s2[1]) != ccw(s1[1],s2[0],s2[1]) && ccw(s1[0],s1[1],s2[0]) != ccw(s1[0],s1[1],s2[1]);
}

bool CollisionsResolver::_circles_intersect(CircleObstacle circle1, CircleObstacle circle2)
{
    float d = sqrt(pow(circle2.position.x - circle1.position.x, 2) + pow(circle2.position.y - circle1.position.y, 2));
    return d <= circle1.radius + circle2.radius;
}

bool CollisionsResolver::_rects_intersect(RectObstacle rect1, RectObstacle rect2)
{
    if (_point_in_rect(rect1.position, rect2) || _point_in_rect(rect2.position, rect1))
        return true; // Checks if a rect is completely inside the other one.

    std::vector<SegmentObstacle> segments1 = rect1.get_segments();
    std::vector<SegmentObstacle> segments2 = rect2.get_segments();
    for(int s1_i = 0; s1_i < segments1.size(); s1_i++)
    { // if not, check if segments intersect
        for(int s2_i = 0; s2_i < segments2.size(); s2_i++)
        {
            if(_segments_intersect(segments1[s1_i], segments2[s2_i]))
                return true;
        }
    }

    return false;
}

bool CollisionsResolver::_segment_intersects_rect(SegmentObstacle segment, RectObstacle rect)
{
    if(_point_in_rect(segment.position, rect))
        return true; // Checks if the segment center is inside the rect.

    std::vector<SegmentObstacle> rect_segments = rect1.get_segments();
    for(int rs_i = 0; rs_i < segments1.size(); rs_i++)
    {
        if(_segments_intersect(segment, rect_segments[rs_i]))
            return true;
    }

    return false;
}

bool CollisionsResolver::_segment_intersects_circle(SegmentObstacle segment, CircleObstacle circle)
{
    RectObstacle new_rect = RectObstacle(segment.position, circle.radius * 2, segment.length + 2 * circle.radius);
    return _rect_intersects_circle(new_rect, circle);
}

bool CollisionsResolver::_point_in_rect(Point point, RectObstacle rect)
{
    float phi = atan2(point.y - rect.position.y, point.x - rect.position.x);
    if(phi < 0) phi += 2*M_PI;

    float a = rect.position.a;
    float d = sqrt(pow(point.x - rect.position.x, 2) + pow(point.y - rect.position.y, 2));

    Point local_point = Point(d * cos(phi - a), d * sin(phi - a));
    return (- rect.width  / 2.0 <= local_point[0] && local_point[0] <= rect.width  / 2.0) && \
           (- rect.height / 2.0 <= local_point[1] && local_point[1] <= rect.height / 2.0);
}

bool CollisionsResolver::_point_in_circle(Point point, CircleObstacle circle)
{
    float d = sqrt(pow(point.x - circle.position.x, 2) + pow(point.y - circle.position.y, 2));
    return d <= circle.radius;
}

bool CollisionsResolver::_rect_intersects_circle(RectObstacle rect, CircleObstacle circle)
{
    RectObstacle new_rect = RectObstacle(rect.position, rect.width + circle.radius * 2, rect.height + circle.radius * 2);
    return _point_in_rect(circle.position, new_rect);
}
