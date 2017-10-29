#ifndef POS_CONVERTOR_H
#define POS_CONVERTOR_H

#include <algorithm>

/**
 * @brief This class provide functions to convert coordinates between ROS system and the pathfinding one 
 */
class PosConvertor
{
public:
    /**
     * @brief Initialize the convertor. It takes a size in the ROS system and in the pthfinder one of the same object (for example the table's size) to make scales.
     * @param sizeRos The size in ROS system
     * @param sizeMap The size in pathfinding system
     */
    PosConvertor(std::pair<double,double> sizeRos, std::pair<double,double> sizeMap):
        _sizeRos(sizeRos), _sizeMap(sizeMap) {}
    
    /**
     * @brief Converts a coodinate from ROS system to pathfinding system using the scales.
     * @param rosPos The coodinate in ROS system
     * @return The coordinate in pathfinding system
     */
    std::pair<double,double> fromRosToMapPos (const std::pair<double,double>& rosPos) const;
    
    /**
     * @brief Converts a coodinate from pathfinding system to ROS system using the scales.
     * @param mapPos The coodinate in pathfinding system
     * @return The coordinate in ROS system
     */
    std::pair<double,double> fromMapToRosPos (const std::pair<double,double>& mapPos) const;
    
private:
    std::pair<double,double> _sizeRos, _sizeMap;
};

#endif // POS_CONVERTOR_H
