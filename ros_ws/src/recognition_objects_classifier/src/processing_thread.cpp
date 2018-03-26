#include "processing_thread.h"
#include "main_thread.h"
#include <iostream>

void ProcessingThread::start()
{
    processing_thread_ = std::thread(&ProcessingThread::thread_function, this);
}

void ProcessingThread::thread_function()
{
    while(!thread_stopped_) {
        // wait until main thread notification
        std::unique_lock<std::mutex> lk(mutex_);
        cv_.wait(lk, [this]{return ready_;});

        // notif received
        ready_ = false;
        processed_ = false;


        // process data
        for(unsigned int i = start_idx_; i < start_idx_ + length_; i++) {
            //std::cout << "Processing idx " << i << std::endl << std::flush;
        }

        // unlocks guard and notifies main thread we finished
        processed_ = true;
        lk.unlock();
        cv_.notify_one();
    }
}

void ProcessingThread::join()
{
    processing_thread_.join();
}
void ProcessingThread::notify(unsigned int start_idx, unsigned int length)
{
    // changes condition variable and notify so the thread can wake up
    {
        std::lock_guard <std::mutex> lk(mutex_);
        start_idx_ = start_idx;
        length_ = length;
        ready_ = true;
        cv_.notify_one();
    }


}


void ProcessingThread::wait_processing()
{
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait(lk, [this]{return processed_;});
}

void ProcessingThread::stop()
{
    thread_stopped_ = true;
}