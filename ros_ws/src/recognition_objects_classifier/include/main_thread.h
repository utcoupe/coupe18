
#ifndef RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H
#define RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H

#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <processing_belt_interpreter/BeltRects.h>

#include "map_objects.h"
#include "processing_thread.h"
#include "markers_publisher.h"


struct Point {
    float x;
    float y;
    bool is_map;
};

const std::string PUB_TOPIC = "/recognition/objects_classifier/objects";

const float PUB_FREQ = 10.0;

const int SENSORS_NBR = 6;
const int MAX_POINTS = 1000; // maximum number of points when the rects are discretized (approximate)
// 1000 =~ 1.4 m range threshold

const int THREADS_NBR = 6;

// discretization steps (m)
const float STEP_X = 0.01;
const float STEP_Y = 0.01;

// minimum fraction of a rect to be in map for it to be considered static
const float MIN_MAP_FRAC = 0.5;

// if the absolute time diff between the received time and the header time is
// greater than this (s), adjusts the header time
const float TIME_DIFF_MAX = 0.05;

class MainThread {
protected:
    ros::NodeHandle &nh_;

    // classified lists
    std::vector<processing_belt_interpreter::RectangleStamped> map_rects_;
    std::vector<processing_belt_interpreter::RectangleStamped> unknown_rects_;

    // protects the published lists
    std::mutex lists_mutex_;

    // to publish classified lists
    ros::Publisher pub_;
    ros::Timer timer_;

    // shared list of points to process (classify)
    Point points_[SENSORS_NBR * MAX_POINTS];
    std::vector<std::unique_ptr<ProcessingThread>> threads_;

    // static objects
    MapObjects map_objects_;

    MarkersPublisher markers_publisher_;

    // transforms objects into tf /map
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tl_;

    void pub_loop(const ros::TimerEvent &);

public:
    MainThread(ros::NodeHandle &nh);

    ~MainThread();

    void process_rects(processing_belt_interpreter::BeltRects &rects);
};


#endif //RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H
