#include <memory>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory_map/MapGet.h>
#include <recognition_objects_classifier/ClassifiedObjects.h>

#include "main_thread.h"


void MainThread::classify_rects(processing_belt_interpreter::BeltRects &rects) {

    {
        std::lock_guard<std::mutex> lk(lists_mutex_);
        classified_objects_.map_rects.clear();
        classified_objects_.unknown_rects.clear();
    }

    if (rects.rects.empty())
        return;

    double time = ros::Time::now().toSec();
    double step_x, step_y;

    int end_idx[rects.rects.size()];

    unsigned int rect_idx = 0;
    unsigned int point_idx = 0;

    geometry_msgs::TransformStamped transform_stamped;

    geometry_msgs::PointStamped point_static_frame, point_map_frame;

    for (auto &rect : rects.rects) {

        std::tie(step_x, step_y) = compute_division_steps(rect);

        if (!fetch_transform_and_adjust_stamp(rect, transform_stamped)) {

            end_idx[rect_idx] = point_idx - 1;
            rect_idx++;
            continue;
        }

        point_static_frame.header = rect.header;
        for (float x = rect.x - rect.w / 2; x <= rect.x + rect.w / 2; x += step_x) {
            point_static_frame.point.x = x;

            for (float y = rect.y - rect.h / 2; y <= rect.y + rect.h / 2; y += step_y) {
                point_static_frame.point.y = y;

                tf2::doTransform(point_static_frame, point_map_frame, transform_stamped);

                this->points_[point_idx].x = static_cast<float>(point_map_frame.point.x);
                this->points_[point_idx].y = static_cast<float>(point_map_frame.point.y);
                point_idx++;
            }
        }

        end_idx[rect_idx] = point_idx - 1;
        rect_idx++;

        transform_rect(rect, transform_stamped);
    }

    if (point_idx == 0)
        return;

    notify_threads_and_wait(point_idx);

    std::lock_guard<std::mutex> lk(lists_mutex_);

    // populate rect classified arrays

    int running_idx = 0;
    unsigned int nbr_map = 0;
    for (int r = 0; r < rects.rects.size(); r++) {

        nbr_map = 0;
        for (int p = running_idx; p <= end_idx[r]; p++) {
            if (points_[p].is_map)
                nbr_map++;
        }

        float frac_map = (float) nbr_map / (float) (end_idx[r] - running_idx);
        running_idx = end_idx[r] + 1;

        if (frac_map >= MIN_MAP_FRAC) {
            classified_objects_.map_rects.push_back(rects.rects[r]);
        } else {
            classified_objects_.unknown_rects.push_back(rects.rects[r]);
        }
    }

    time = ros::Time::now().toSec() - time;

    ROS_DEBUG("Took %f secs to process %lu rects, %d points", time, rects.rects.size(), point_idx);
}

std::pair<float, float> MainThread::compute_division_steps(
        const processing_belt_interpreter::RectangleStamped &rect) {

    auto samples_x = static_cast<unsigned int>(rect.w / STEP_X);
    auto samples_y = static_cast<unsigned int>(rect.h / STEP_Y);

    float step_x = STEP_X;
    float step_y = STEP_Y;

    if (samples_x * samples_y > MAX_POINTS) {
        step_x = static_cast<float>(rect.w / sqrt(MAX_POINTS));
        step_y = static_cast<float>(rect.h / sqrt(MAX_POINTS));

        samples_x = static_cast<unsigned int>(rect.w / step_x);
        samples_y = static_cast<unsigned int>(rect.h / step_y);
    }

    if (samples_x < 2) {
        step_x = rect.w;
    }
    if (samples_y < 2) {
        step_y = rect.h;
    }

    return {step_x, step_y};
}

bool MainThread::fetch_transform_and_adjust_stamp(
        processing_belt_interpreter::RectangleStamped &rect,
        geometry_msgs::TransformStamped &transform_out) {

    ros::Time common_time;
    std::string err_msg;

    try {
        tf_buffer_._getLatestCommonTime(
                tf_buffer_._lookupFrameNumber("map"),
                tf_buffer_._lookupFrameNumber(rect.header.frame_id),
                common_time, &err_msg
        );

        // adjust the timestamp of the rect if it's a bit later than the last transform
        if (rect.header.stamp > common_time &&
            fabs(rect.header.stamp.toSec() - common_time.toSec()) < TIME_DIFF_MAX) {
            rect.header.stamp = common_time;
        }

        transform_out = tf_buffer_.lookupTransform("map", rect.header.frame_id, rect.header.stamp);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    return true;
}


void MainThread::transform_rect(processing_belt_interpreter::RectangleStamped &rect,
                                geometry_msgs::TransformStamped &transform) {

    geometry_msgs::PoseStamped pose_static_frame, pose_map_frame;

    pose_static_frame.header = rect.header;
    pose_static_frame.pose.position.x = rect.x;
    pose_static_frame.pose.position.y = rect.y;
    pose_static_frame.pose.orientation = tf::createQuaternionMsgFromYaw(rect.a);

    tf2::doTransform(pose_static_frame, pose_map_frame, transform);

    rect.x = static_cast<float>(pose_map_frame.pose.position.x);
    rect.y = static_cast<float>(pose_map_frame.pose.position.y);
    rect.a = static_cast<float>(tf::getYaw(pose_map_frame.pose.orientation));
    rect.header.frame_id = transform.child_frame_id;

}

void MainThread::notify_threads_and_wait(int num_points) {

    auto size = static_cast<unsigned int>(ceil((double) num_points / (double) THREADS_NBR));

    unsigned int used_threads = 0;
    for (int t = 0; t < THREADS_NBR; t++) {
        // we finished the list
        if (t * size >= num_points)
            break;

        used_threads++;

        // end of the list
        if (t * size + size >= num_points)
            threads_[t]->notify(t * size, size - 1);
        else
            threads_[t]->notify(t * size, size);
    }

    for (int t = 0; t < used_threads; t++) {
        threads_[t]->wait_processing();
    }
}

void MainThread::classify_lidar_objects(processing_lidar_objects::Obstacles &obstacles) {

    double time = ros::Time::now().toSec();

    std::lock_guard<std::mutex> lk(lists_mutex_);

    classified_objects_.map_circles.clear();
    classified_objects_.unknown_circles.clear();
    classified_objects_.map_segments.clear();
    classified_objects_.unknown_segments.clear();

    recognition_objects_classifier::CircleObstacleStamped circle_s;
    circle_s.header = obstacles.header;

    recognition_objects_classifier::SegmentObstacleStamped segment_s;
    segment_s.header = obstacles.header;

    // remove segments that are inside circles
    for (auto &circle : obstacles.circles) {

        // remove segments that are inside circles
        for (auto it = obstacles.segments.begin(); it != obstacles.segments.end();) {
            if (pow(it->first_point.x - circle.center.x, 2) + pow(it->first_point.y - circle.center.y, 2)
                <= pow(circle.true_radius, 2) &&
                pow(it->last_point.x - circle.center.x, 2) + pow(it->last_point.y - circle.center.y, 2)
                <= pow(circle.true_radius, 2)) {

                it = obstacles.segments.erase(it);
            } else {
                ++it;
            }
        }

        // classify circle
        circle_s.circle = circle;
        if (pow(circle.velocity.x, 2) + pow(circle.velocity.y, 2) + pow(circle.velocity.z, 2) >=
            pow(CIRCLE_SPEED_MAX, 2) || !map_objects_.contains_point(static_cast<float>(circle.center.x),
                                                                     static_cast<float>(circle.center.y))) {

            classified_objects_.unknown_circles.push_back(circle_s);
        } else {
            classified_objects_.map_circles.push_back(circle_s);
        }
    }

    for (auto &segment : obstacles.segments) {
        segment_s.segment = segment;

        if (map_objects_.contains_point(static_cast<float>(segment.first_point.x),
                                        static_cast<float>(segment.first_point.y)) &&
            map_objects_.contains_point(static_cast<float>(segment.last_point.x),
                                        static_cast<float>(segment.last_point.y))) {
            classified_objects_.map_segments.push_back(segment_s);
        } else {
            classified_objects_.unknown_segments.push_back(segment_s);
        }
    }

    time = ros::Time::now().toSec() - time;

    ROS_DEBUG("Took %f secs to process lidar data", time);

}

void MainThread::pub_loop(const ros::TimerEvent &) {
    std::lock_guard<std::mutex> lk(lists_mutex_);

    pub_.publish(classified_objects_);

    if (markers_publisher_.is_connected())
        markers_publisher_.publish_rects(classified_objects_.map_rects, classified_objects_.unknown_rects);
}

MainThread::MainThread(ros::NodeHandle &nh) :
        nh_(nh),
        pub_(nh.advertise<recognition_objects_classifier::ClassifiedObjects>(PUB_TOPIC, 1)),
        timer_(nh_.createTimer(ros::Duration(1.0 / PUB_FREQ), &MainThread::pub_loop, this)),
        tl_(tf_buffer_),
        markers_publisher_(nh),
        map_objects_(nh) {

    map_objects_.fetch_map_objects();

    for (int i = 0; i < THREADS_NBR; i++) {
        threads_.push_back(std::unique_ptr<ProcessingThread>(new ProcessingThread(points_, map_objects_)));
        threads_[i]->start();
    }
}

MainThread::~MainThread() {
    for (auto &thread : threads_) {
        thread->stop();
        thread->join();
    }
}