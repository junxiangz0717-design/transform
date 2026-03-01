
#ifndef TIME_MEASURE_H
#define TIME_MEASURE_H

#include <iostream>
#include <chrono>
#include <string>

class time_measure 
{
public:
    time_measure() {}

    void start() 
    {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    template <typename TimeUnit = std::chrono::milliseconds>
    long long stop() 
    {
        auto end_time = std::chrono::high_resolution_clock::now();
        //milliseconds microseconds nanoseconds
        auto duration = std::chrono::duration_cast<TimeUnit>(end_time - start_time_).count();
        return duration;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};    
#endif