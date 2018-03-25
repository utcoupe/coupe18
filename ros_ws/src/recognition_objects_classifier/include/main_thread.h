
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

const int SENSORS_NBR = 6;
const int MAX_POINTS = 100; // maximum number of points when the rects are discretized

const int THREADS_NBR = 6;

// discretization steps
const float STEP_X = 0.02;
const float STEP_Y = 0.02;

class MainThread
{
protected:
    Point points_[SENSORS_NBR * MAX_POINTS];
    std::vector<std::unique_ptr<ProcessingThread>> threads_;
    std::vector<std::shared_ptr<const Shape>> map_shapes_;

    ros::NodeHandle& nh_;


    void fetch_map_objects();

public:
    MainThread(ros::NodeHandle& nh);

    void process_rects(processing_belt_interpreter::BeltRects& rects);




};


#endif //RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H
