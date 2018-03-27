
#include "main_thread.h"
#include "memory_map/MapGet.h"
#include "recognition_objects_classifier/ClassifiedObjects.h"
#include <memory>

void MainThread::process_rects(processing_belt_interpreter::BeltRects &rects)
{

    double time = ros::Time::now().toSec();

    if(rects.rects.size() == 0)
        return;

    unsigned int point_idx = 0;

    // end indexes for each rect
    int end_idx[rects.rects.size()];
    int rect_idx = 0;

    for(auto it=rects.rects.begin(); it != rects.rects.end(); it++)
    {
        int samples_x = (int)(it->w / STEP_X);
        int samples_y = (int)(it->h / STEP_Y);

        float stepx = STEP_X;
        float stepy = STEP_Y;

        if(samples_x * samples_y > MAX_POINTS)
        {
            stepx = (float)(it->w / sqrt(MAX_POINTS));
            stepy = (float)(it->h / sqrt(MAX_POINTS));

            samples_x = (int)(it->w / stepx);
            samples_y = (int)(it->h / stepy);
        }


        // TODO: handle the case where sampleX < 2 or sampleY < 2

        for(float x = it->x - it->w / 2; x <= it->x + it->w / 2; x += stepx)
        {
            for(float y = it->y - it->h / 2; y <= it->y + it->h / 2; y += stepy)
            {

                this->points_[point_idx].x = x;
                this->points_[point_idx].y = y;
                point_idx++;
            }
        }

        end_idx[rect_idx] = point_idx - 1;

        rect_idx++;
    }

    int size = ceil((double)point_idx / (double)THREADS_NBR);

    int used_threads = 0;
    for(int t = 0; t < THREADS_NBR; t++)
    {
        if(t*size >= point_idx)
            break;

        used_threads ++;

        if(t*size + size >= point_idx)
            threads_[t]->notify(t * size, size - 1);
        else
            threads_[t]->notify(t * size, size);
    }

    for(int t = 0; t < used_threads; t++)
        threads_[t]->wait_processing();


    std::lock_guard<std::mutex> lk(lists_mutex_);

    // clear then populate rect classified arrays
    map_rects_.clear();
    unknown_rects_.clear();

    int running_idx = 0;
    int nbr_map = 0;
    for(int r = 0; r < rects.rects.size(); r++)
    {
        nbr_map = 0;
        for(int p = running_idx; p <= end_idx[r]; p++)
        {
            if(points_[p].is_map)
                nbr_map++;
        }

        float frac_map = (float)nbr_map / (float)(end_idx[r] - running_idx);
        running_idx = end_idx[r] + 1;

       if(frac_map >= MIN_MAP_FRAC)
       {
            map_rects_.push_back(rects.rects[r]);
       }
       else
       {
            unknown_rects_.push_back(rects.rects[r]);
       }
    }

    time = ros::Time::now().toSec() - time;

    ROS_DEBUG("Took %f secs to process %d rects, %d points", time, (int)rects.rects.size(), point_idx);


}

void MainThread::pub_loop(const ros::TimerEvent&)
{
    std::lock_guard<std::mutex> lk(lists_mutex_);

    recognition_objects_classifier::ClassifiedObjects msg;

    msg.map_rects = map_rects_;
    msg.unknown_rects = unknown_rects_;

    pub_.publish(msg);
}

MainThread::MainThread(ros::NodeHandle &nh) :
    nh_(nh),
    pub_(nh.advertise<recognition_objects_classifier::ClassifiedObjects>(PUB_TOPIC, 1)),
    timer_(nh_.createTimer(ros::Duration(1.0/PUB_FREQ), &MainThread::pub_loop, this)),
    map_objects_(nh)
{
    map_objects_.fetch_map_objects();

    for(int i = 0; i < THREADS_NBR; i++)
    {
        threads_.push_back(std::unique_ptr<ProcessingThread>(new ProcessingThread(points_, map_objects_)));
        threads_[i]->start();
    }


}