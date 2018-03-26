
#ifndef RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H
#define RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H

#include "processing_belt_interpreter/BeltRects.h"
#include "processing_thread.h"
#include "ros/ros.h"
#include "shapes.h"
#include <vector>



struct Point {
    float x;
    float y;
    bool is_map;
};

const std::string MAP_GET_SERVICE = "/memory/map/get";
const std::string MAP_OBJECTS = "/terrain/walls/layer_belt/*";
const std::string PUB_TOPIC = "/recognition/objects_classifier/objects";

const float PUB_FREQ = 10.0;

const int SENSORS_NBR = 6;
const int MAX_POINTS = 100; // maximum number of points when the rects are discretized

const int THREADS_NBR = 6;

// discretization steps
const float STEP_X = 0.02;
const float STEP_Y = 0.02;

// minimum fraction of a rect to be in map for it to be considered static
const float MIN_MAP_FRAC = 0.5;

class MainThread
{
protected:
    Point points_[SENSORS_NBR * MAX_POINTS];
    std::vector<std::unique_ptr<ProcessingThread>> threads_;
    std::vector<std::shared_ptr<const Shape>> map_shapes_;

    std::vector<processing_belt_interpreter::RectangleStamped> map_rects_;
    std::vector<processing_belt_interpreter::RectangleStamped> unknown_rects_;

    ros::NodeHandle& nh_;

    ros::Publisher pub_;
    ros::Timer timer_;

    // protects the published lists
    std::mutex lists_mutex_;

    void fetch_map_objects();
    void pub_loop(const ros::TimerEvent&);

public:
    MainThread(ros::NodeHandle& nh);

    void process_rects(processing_belt_interpreter::BeltRects& rects);




};


#endif //RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H
