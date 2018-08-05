#include "engine_shapes_attrib.h"


// Point class
Point::Point() {
    Point::x = 0;
    Point::y = 0;
}
Point::Point(float x, float y)
{
    Point::x = x;
    Point::y = y;
}


// Position class
Position::Position():Point()
{
    Position::a = 0;
}
Position::Position(float x, float y, float a = 0):Point(x, y)
{
    Position::a = a;
}
