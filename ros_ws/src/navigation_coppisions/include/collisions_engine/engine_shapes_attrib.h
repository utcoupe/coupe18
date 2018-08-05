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


#endif