
#ifndef RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H
#define RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H

#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <processing_belt_interpreter/BeltRects.h>
#include "processing_lidar_objects/Obstacles.h"
#include <recognition_objects_classifier/ClassifiedObjects.h>

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

// maximum number of points created per rect
const int MAX_POINTS = 500;

const int THREADS_NBR = 6;

// discretization steps (m)
const float STEP_X = 0.02;
const float STEP_Y = 0.02;

// minimum fraction of a rect to be in map for it to be considered static
const float MIN_MAP_FRAC = 0.5;

// if the absolute time diff between the received time and the header time is
// greater than this (s), adjusts the header time (for rects)
const float TIME_DIFF_MAX = 0.05;

// if a circle has a velocity above this value, it is considered unknown
const float CIRCLE_SPEED_MAX = 1.0;

class MainThread {
protected:
    ros::NodeHandle &nh_;

    recognition_objects_classifier::ClassifiedObjects classified_objects_;
    std::mutex lists_mutex_;

    ros::Publisher pub_;
    ros::Timer timer_;

    Point points_[SENSORS_NBR * MAX_POINTS];
    std::vector<std::pair<int, geometry_msgs::TransformStamped>> rects_transforms_;
    std::vector<std::unique_ptr<ProcessingThread>> threads_;

    MapObjects map_objects_;

    MarkersPublisher markers_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tl_;

    void pub_loop(const ros::TimerEvent &);

    std::pair<float, float> compute_division_steps(
            const processing_belt_interpreter::RectangleStamped &rect);

    bool fetch_transform_and_adjust_stamp(
            processing_belt_interpreter::RectangleStamped &rect,
            geometry_msgs::TransformStamped &transform_out);

    void transform_rect(
            processing_belt_interpreter::RectangleStamped &rect,
            geometry_msgs::TransformStamped &transform);

    void notify_threads_and_wait(int num_points);

public:
    MainThread(ros::NodeHandle &nh);

    ~MainThread();

    void classify_rects(processing_belt_interpreter::BeltRects &rects);

    void classify_lidar_objects(processing_lidar_objects::Obstacles &obstacles);
};


#endif //RECOGNITION_OBJECTS_CLASSIFIER_MAIN_THREAD_H
