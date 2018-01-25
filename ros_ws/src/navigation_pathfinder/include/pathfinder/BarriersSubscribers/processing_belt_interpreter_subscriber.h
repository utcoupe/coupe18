#ifndef PROCESSING_BELT_INTERPRETER_SUBSCRIBER
#define PROCESSING_BELT_INTERPRETER_SUBSCRIBER

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"

namespace Processing {
    class BeltInterpreterSubscriber : public AbstractBarriersSubscriber
    {
    public:
        BeltInterpreterSubscriber();
        
        bool hasBarrier(const Point& pos) const;
        
    private:
        //
    };
}

#endif // PROCESSING_BELT_INTERPRETER_SUBSCRIBER
