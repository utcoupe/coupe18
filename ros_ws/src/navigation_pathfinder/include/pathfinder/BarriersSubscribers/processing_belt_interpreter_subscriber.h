#ifndef PROCESSING_BELT_INTERPRETER_SUBSCRIBER
#define PROCESSING_BELT_INTERPRETER_SUBSCRIBER

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "processing_belt_interpreter/BeltRects.h"
#include "processing_belt_interpreter/RectangleStamped.h"

#include <vector>

namespace Processing {
    class BeltInterpreterSubscriber : public AbstractBarriersSubscriber
    {
    public:
        BeltInterpreterSubscriber(const double& safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
    private:
        typedef processing_belt_interpreter::RectangleStamped Rectangle;
        std::vector<Rectangle> lastRectangles;
        
        void rectsFilteredTopicCallback(const processing_belt_interpreter::BeltRects::ConstPtr& msg);
        void addRects(const std::vector<Rectangle>& rects);
        bool isInsideRect(const geometry_msgs::Pose2D& pos, const Rectangle& rect) const;
    };
}

#endif // PROCESSING_BELT_INTERPRETER_SUBSCRIBER
