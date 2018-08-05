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


// Velocity class
Velocity::Velocity(float object_width, float object_height, float linear, float angular)
{
    Velocity::object_width = object_width;
    Velocity::object_height = object_height;

    Velocity::linear = linear;
    Velocity::angular = angular;
}