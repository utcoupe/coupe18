
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

bool MarkersPublisher::is_connected() {
    return pub_.getNumSubscribers() > 0;
}