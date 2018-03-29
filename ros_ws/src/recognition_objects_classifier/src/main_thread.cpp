#include <memory>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory_map/MapGet.h>
#include <recognition_objects_classifier/ClassifiedObjects.h>

#include "main_thread.h"


void MainThread::process_rects(processing_belt_interpreter::BeltRects &rects) {

    if (rects.rects.empty())
        return;

    double time = ros::Time::now().toSec();
    double step_y;
    double step_x;

    int end_idx[rects.rects.size()];

    unsigned int rect_idx = 0;
    unsigned int point_idx = 0;
    unsigned int samples_x, samples_y;

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PointStamped point_init;
    geometry_msgs::PointStamped point_map;

    ros::Time common_time;
    std::string err_msg;

    for (auto &rect : rects.rects) {

        // figure out the discretization steps
        samples_x = static_cast<unsigned int>(rect.w / STEP_X);
        samples_y = static_cast<unsigned int>(rect.h / STEP_Y);

        step_x = STEP_X;
        step_y = STEP_Y;

        if (samples_x * samples_y > MAX_POINTS) {
            step_x = rect.w / sqrt(MAX_POINTS);
            step_y = rect.h / sqrt(MAX_POINTS);

            samples_x = static_cast<unsigned int>(rect.w / step_x);
            samples_y = static_cast<unsigned int>(rect.h / step_y);
        }

        // TODO: handle the case where sample_x < 2 or sample_y < 2

        // get the transform from the static tf to /map
        try {
            // adjust the timestamp of the rect if it's a bit later than the last transform
            tf_buffer_._getLatestCommonTime(
                    tf_buffer_._lookupFrameNumber("map"),
                    tf_buffer_._lookupFrameNumber(rect.header.frame_id),
                    common_time, &err_msg
            );

            if (rect.header.stamp > common_time &&
                fabs(rect.header.stamp.toSec() - common_time.toSec()) < TIME_DIFF_MAX) {
                rect.header.stamp = common_time;
            }

            transformStamped = tf_buffer_.lookupTransform("map", rect.header.frame_id, rect.header.stamp);

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            end_idx[rect_idx] = point_idx - 1;
            rect_idx++;
            continue;
        }

        point_init.header = rect.header;

        for (float x = rect.x - rect.w / 2; x <= rect.x + rect.w / 2; x += step_x) {
            point_init.point.x = x;

            for (float y = rect.y - rect.h / 2; y <= rect.y + rect.h / 2; y += step_y) {
                point_init.point.y = y;

                tf2::doTransform(point_init, point_map, transformStamped);

                this->points_[point_idx].x = static_cast<float>(point_map.point.x);
                this->points_[point_idx].y = static_cast<float>(point_map.point.y);
                point_idx++;
            }
        }

        end_idx[rect_idx] = point_idx - 1;

        rect_idx++;
    }

    if (point_idx == 0)
        return;

    auto size = static_cast<unsigned int>(ceil((double) point_idx / (double) THREADS_NBR));

    unsigned int used_threads = 0;
    for (int t = 0; t < THREADS_NBR; t++) {
        // we finished the list
        if (t * size >= point_idx)
            break;

        used_threads++;

        // end of the list
        if (t * size + size >= point_idx)
            threads_[t]->notify(t * size, size - 1);
        else
            threads_[t]->notify(t * size, size);
    }

    // wait for the threads to finish
    for (auto &thread : threads_) {
        thread->wait_processing();
    }

    std::lock_guard<std::mutex> lk(lists_mutex_);

    // clear then populate rect classified arrays
    map_rects_.clear();
    unknown_rects_.clear();

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
            map_rects_.push_back(rects.rects[r]);
        } else {
            unknown_rects_.push_back(rects.rects[r]);
        }
    }

    time = ros::Time::now().toSec() - time;

    ROS_DEBUG("Took %f secs to process %lu rects, %d points", time, rects.rects.size(), point_idx);
}

void MainThread::pub_loop(const ros::TimerEvent &) {
    std::lock_guard<std::mutex> lk(lists_mutex_);

    recognition_objects_classifier::ClassifiedObjects msg;

    msg.map_rects = map_rects_;
    msg.unknown_rects = unknown_rects_;

    pub_.publish(msg);
}

MainThread::MainThread(ros::NodeHandle &nh) :
        nh_(nh),
        pub_(nh.advertise<recognition_objects_classifier::ClassifiedObjects>(PUB_TOPIC, 1)),
        timer_(nh_.createTimer(ros::Duration(1.0 / PUB_FREQ), &MainThread::pub_loop, this)),
        tl_(tf_buffer_),
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