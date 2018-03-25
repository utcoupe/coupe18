
#ifndef RECOGNITION_OBJECTS_CLASSIFIER_PROCESSING_THREAD_H
#define RECOGNITION_OBJECTS_CLASSIFIER_PROCESSING_THREAD_H

#include <thread>
#include <mutex>
#include <condition_variable>

struct Point;

class ProcessingThread
{
protected:
    std::thread processing_thread_;
    bool thread_stopped_;

    unsigned int start_idx_;
    unsigned int length_;
    Point* points_;

    // to pause the thread
    std::mutex mutex_;
    std::condition_variable cv_;

    // indicates the processing_thread to start processing
    bool ready_;

    // indicates the main thread that the processing is finished
    bool processed_;

    void thread_function();
public:
    void start();
    void stop();
    void join();
    void notify(unsigned int start_idx, unsigned int length);
    void wait_processing();

    ProcessingThread(Point* points) :
        points_(points),
        thread_stopped_(false),
        ready_(false),
        processed_(false) {}

    ProcessingThread(const ProcessingThread& other) :
        points_(other.points_),
        thread_stopped_(other.thread_stopped_),
        ready_(other.ready_),
        processed_(other.processed_) {}
};

#endif //RECOGNITION_OBJECTS_CLASSIFIER_PROCESSING_THREAD_H
