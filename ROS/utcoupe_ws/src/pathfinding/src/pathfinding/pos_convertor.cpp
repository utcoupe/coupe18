#include "pathfinding/pos_convertor.hpp"

using namespace std;

pair<double,double> PosConvertor::fromRosToMapPos (const pair<double,double>& rosPos) const
{
    double x, y;
    x = rosPos.first * (_sizeMap.first / _sizeRos.first);
    y = rosPos.second * (_sizeMap.second / _sizeRos.second);
    
    if (_invertedY)
        y = _sizeMap.second - y;
    
    return make_pair(x, y);
}


pair<double,double> PosConvertor::fromMapToRosPos (const pair<double,double>& mapPos) const
{
    double x, y;
    x = mapPos.first * (_sizeRos.first / _sizeMap.first);
    y = mapPos.second * (_sizeRos.second / _sizeMap.second);
    
    if (_invertedY)
        y = _sizeRos.second - y;
    
    return make_pair(x, y);
}

void PosConvertor::setSizes(std::pair<double, double> sizeRos, std::pair<double, double> sizeMap)
{
    _sizeRos = sizeRos;
    _sizeMap = sizeMap;
}
