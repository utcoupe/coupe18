
#ifndef PROJECT_MAP_OBJECTS_H
#define PROJECT_MAP_OBJECTS_H

#include "shapes.h"

const std::string MAP_GET_SERVICE = "/memory/map/get";
const std::string MAP_OBJECTS = "/terrain/walls/layer_belt/*";

class MapObjects
{
protected:
    std::vector<std::shared_ptr<const Shape>> map_shapes_;

public:
    void fetch_map_objects();

};

#endif //PROJECT_MAP_OBJECTS_H
