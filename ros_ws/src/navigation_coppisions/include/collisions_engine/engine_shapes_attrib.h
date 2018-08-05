#ifndef ENGINE_SHAPES_ATTRIB_H
#define ENGINE_SHAPES_ATTRIB_H


enum class ObstacleType 
{
    OBSTACLE,
    POINT,
    SEGMENT,
    CIRCLE,
    RECT
};


class Point
{
private:
    /* data */
public:
    ObstacleType type = ObstacleType::POINT;
    float x, y;
    Point();
    Point(float x, float y);
    ~Point();
};


class Position: public Point
{
private:
    /* data */
public:
    float a;
    Position();
    Position(float x, float y, float a);
    ~Position();
};


class Velocity
{
public:
    float object_width, object_height; // Used for when VelocityCheckZones will be created.
    float linear, angular;

    Velocity();
    Velocity(float width, float height, float linear = 0, float angular = 0);
    ~Velocity();
};

#endif