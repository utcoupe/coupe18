
#include "main_thread.h"
#include "memory_map/MapGet.h"
#include <memory>

void MainThread::fetch_map_objects()
{
    ros::ServiceClient client = nh_.serviceClient<memory_map::MapGet>(MAP_GET_SERVICE);

    client.waitForExistence();

    memory_map::MapGet srv;

    srv.request.request_path = MAP_OBJECTS;

    if(client.call(srv) && srv.response.success)
    {

    }
    else
    {
        ROS_ERROR("Failed to contact memory_map, static objects not fetched");
    }
}

void MainThread::process_rects(processing_belt_interpreter::BeltRects &rects)
{

    if(rects.rects.size() == 0)
        return;

    unsigned int i = 0;

    // end indexes for each rect
    int end_idx[rects.rects.size()];
    int rect_idx = 0;

    for(auto it=rects.rects.begin(); it != rects.rects.end(); it++)
    {
        rect_idx ++;
        int samples_x = (int)(it->w / STEP_X);
        int samples_y = (int)(it->h / STEP_Y);

        float stepx = STEP_X;
        float stepy = STEP_Y;

        if(samples_x * samples_y > MAX_POINTS)
        {
            stepx = (float)(sqrt(it->w * it->h / MAX_POINTS));
            stepy = stepx;

            samples_x = (int)(it->w / stepx);
            samples_y = (int)(it->h / stepy);
        }


        // TODO: handle the case where sampleX < 2 or sampleY < 2

        for(float x = it->x - it->w / 2; x <= it->x + it->w / 2; x += stepx)
        {
            for(float y = it->y - it->h / 2; y <= it->y + it->h / 2; y += stepy)
            {

                this->points_[i].x = x;
                this->points_[i].y = y;
                i++;
            }
        }

        end_idx[rect_idx] = i - 1;
    }

    ROS_INFO("num points : %d", i);

    int size = (int)ceil((double)i / (double)THREADS_NBR);

    int used_threads = 0;
    for(int t = 0; t < THREADS_NBR; t++)
    {
        if(t*size >= i)
            break;

        used_threads ++;

        threads_[t]->notify(t * size, size);
    }

    for(int t = 0; t < used_threads; t++)
        threads_[t]->wait_processing();

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

        //TODO: classify rect
    }

}

MainThread::MainThread(ros::NodeHandle &nh) : nh_(nh)
{
    for(int i = 0; i < THREADS_NBR; i++)
    {
        threads_.push_back(std::unique_ptr<ProcessingThread>(new ProcessingThread(points_)));
        threads_[i]->start();
    }
}