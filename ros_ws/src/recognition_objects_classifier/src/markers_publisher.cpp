
#include <tf/transform_datatypes.h>

#include "markers_publisher.h"

void MarkersPublisher::publish_rects(std::vector<processing_belt_interpreter::RectangleStamped> map_rects,
                                     std::vector<processing_belt_interpreter::RectangleStamped> unknown_rects) {

    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.CUBE;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.lifetime = ros::Duration(LIFETIME);
    marker.pose.position.z = Z_POS;
    marker.scale.z = Z_SCALE;
    marker.ns = "rects";

    std::vector<processing_belt_interpreter::RectangleStamped> *both_lists[2] = {&map_rects, &unknown_rects};

    for (auto &list : both_lists) {
        for (auto &rect : *list) {
            marker.pose.position.x = rect.x;
            marker.pose.position.y = rect.y;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(rect.a);
            marker.scale.x = rect.w;
            marker.scale.y = rect.h;
            marker.header = rect.header;

            pub_.publish(marker);
        }

        marker.color.r = 1.0;
        marker.color.g = 0.0;
    }
}

void MarkersPublisher::publish_circles(std::vector<recognition_objects_classifier::CircleObstacleStamped> map_circles,
                                       std::vector<recognition_objects_classifier::CircleObstacleStamped> unknown_circles) {

    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.CYLINDER;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.lifetime = ros::Duration(LIFETIME);
    marker.pose.position.z = Z_POS;
    marker.scale.z = Z_SCALE;
    marker.ns = "circles";

    std::vector<recognition_objects_classifier::CircleObstacleStamped> *both_lists[2] = {&map_circles,
                                                                                         &unknown_circles};

    for (auto &list : both_lists) {
        for (auto &circle : *list) {
            marker.pose.position.x = circle.circle.center.x;
            marker.pose.position.y = circle.circle.center.y;
            marker.scale.x = circle.circle.true_radius;
            marker.scale.y = circle.circle.true_radius;
            marker.header = circle.header;

            pub_.publish(marker);
        }

        marker.color.r = 1.0;
        marker.color.g = 0.0;
    }
}

void MarkersPublisher::publish_segments(
        std::vector<recognition_objects_classifier::SegmentObstacleStamped> map_segments,
        std::vector<recognition_objects_classifier::SegmentObstacleStamped> unknown_segments) {


    std::vector<geometry_msgs::Point> points;

    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.LINE_LIST;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.lifetime = ros::Duration(LIFETIME);
    marker.pose.position.z = Z_POS;
    marker.ns = "segments";

    geometry_msgs::Point point;

    std::vector<recognition_objects_classifier::SegmentObstacleStamped> *both_lists[2] = {&map_segments,
                                                                                          &unknown_segments};

    for (auto &list : both_lists) {
        points.clear();



        for (auto &segment : *list) {
            point.x = segment.segment.first_point.x;
            point.y = segment.segment.first_point.y;
            points.push_back(point);

            point.x = segment.segment.last_point.x;
            point.y = segment.segment.last_point.y;
            points.push_back(point);
        }

        if(!list->empty()) {
            marker.header = list->at(0).header;
            pub_.publish(marker);
        }

        marker.color.r = 1.0;
        marker.color.g = 0.0;
    }


}

bool MarkersPublisher::is_connected() {
    return pub_.getNumSubscribers() > 0;
}