#ifndef DYNAMIC_BARRIERS_MANAGER_H
#define DYNAMIC_BARRIERS_MANAGER_H

#include "pathfinder/point.h"
#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "pathfinder/pos_convertor.h"

#include "geometry_msgs/Pose2D.h"

#include <memory>
#include <vector>

class DynamicBarriersManager
{
public:
    DynamicBarriersManager();
    
    bool hasBarriers(const geometry_msgs::Pose2D& pos);
    bool hasBarriers(const Point& pos);
    void addBarrierSubscriber(std::unique_ptr<AbstractBarriersSubscriber>&& subscriber);
    void setConvertor(std::shared_ptr<PosConvertor> convertor);
    
private:
    std::vector< std::unique_ptr<AbstractBarriersSubscriber> > subscribers;
    std::shared_ptr<PosConvertor> _convertor;
    
    /**
     * Converts a position from the inside referential and type to the outside ones.
     * @param pos The position in the inside referential and type.
     * @return The position in the outside referential and type.
     */
    geometry_msgs::Pose2D pointToPose2D(const Point& pos) const;
};

#endif // DYNAMIC_BARRIERS_MANAGER_H
