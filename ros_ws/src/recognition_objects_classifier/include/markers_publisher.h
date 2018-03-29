
#ifndef PROJECT_MARKERS_PUBLISHER_H
#define PROJECT_MARKERS_PUBLISHER_H

#include <vector>
#include <string>

#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

#include "processing_belt_interpreter/RectangleStamped.h"

const std::string MARKERS_TOPIC = "/visualization_markers/objects";
const float LIFETIME = 0.1;
const float Z_POS = 0.2;
const float Z_SCALE = 0.2;

class MarkersPublisher {
protected:
    ros::Publisher pub_;

public:
    void publish_rects(std::vector<processing_belt_interpreter::RectangleStamped> map_rects,
                       std::vector<processing_belt_interpreter::RectangleStamped> unknown_rects);

    bool is_connected();

    MarkersPublisher(ros::NodeHandle &nh) :
            pub_(nh.advertise<visualization_msgs::Marker>(MARKERS_TOPIC, 6)) {}
};

#endif //PROJECT_MARKERS_PUBLISHER_H
