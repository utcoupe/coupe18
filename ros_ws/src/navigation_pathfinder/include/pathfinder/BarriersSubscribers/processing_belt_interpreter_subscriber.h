#ifndef PROCESSING_BELT_INTERPRETER_SUBSCRIBER
#define PROCESSING_BELT_INTERPRETER_SUBSCRIBER

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "processing_belt_interpreter/BeltFiltered.h"
#include "processing_belt_interpreter/RectangleStamped.h"

#include <vector>

namespace Processing {
    class BeltInterpreterSubscriber : public AbstractBarriersSubscriber
    {
    public:
        BeltInterpreterSubscriber();
        
        bool hasBarrier(const Point& pos) const;
        void rectsFilteredTopicCallback(const processing_belt_interpreter::BeltFiltered::ConstPtr& msg);
        
    private:
        typedef processing_belt_interpreter::RectangleStamped Rectangle;
        std::vector<Rectangle> lastRectangles;
        
        void addRects(const std::vector<Rectangle>& rects);
        
        bool isInsideRect(const Point& pos, const Rectangle& rect) const;
    };
}

#endif // PROCESSING_BELT_INTERPRETER_SUBSCRIBER
