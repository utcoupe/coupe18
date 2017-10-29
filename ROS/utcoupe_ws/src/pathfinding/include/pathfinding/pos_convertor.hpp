#ifndef POS_CONVERTOR_H
#define POS_CONVERTOR_H

#include <algorithm>

class PosConvertor
{
public:
    PosConvertor(std::pair<double,double> sizeRos, std::pair<double,double> sizeMap):
        _sizeRos(sizeRos), _sizeMap(sizeMap) {}
    
    std::pair<double,double> fromRosToMapPos (const std::pair<double,double>& rosPos) const;
    std::pair<double,double> fromMapToRosPos (const std::pair<double,double>& mapPos) const;
    
private:
    std::pair<double,double> _sizeRos, _sizeMap;
};

#endif // POS_CONVERTOR_H
