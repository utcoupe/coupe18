#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include <ros/console.h>

#include "pathfinder/point.h"
#include "pathfinder/dynamic_barriers_manager.h"

#include <memory>
#include <vector>

#include <SFML/Graphics/Image.hpp>

class MapStorage
{
public:
    typedef std::vector<std::vector<bool> > Vect2DBool;
    //typedef std::vector<std::vector<unsigned short> > DynamicBarriers;
    
    MapStorage() {}
    
    Vect2DBool loadAllowedPositionsFromFile(const std::string& fileName);
    void saveMapToFile(const std::string& fileName, const Vect2DBool& allowedPos, std::shared_ptr<DynamicBarriersManager> dynBarriersMng, const std::vector<Point>& path, const std::vector<Point>& smoothPath);

private:
    const sf::Color ALLOWED_POS_COLOR       = sf::Color(255, 255, 255);
    const sf::Color NOT_ALLOWED_POS_COLOR   = sf::Color(0, 0, 0);
    const sf::Color DYN_BARRIER_COLOR       = sf::Color(255, 0, 0);
    const sf::Color PATH_COLOR              = sf::Color(0, 0, 250);
    const sf::Color SMOOTH_PATH_COLOR       = sf::Color(0, 255, 0);
    
    void drawPath(sf::Image& image, const Point& pA, const Point& pB, const sf::Color& color);
};

#endif // MAP_STORAGE_H
