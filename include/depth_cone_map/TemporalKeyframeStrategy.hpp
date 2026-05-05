#ifndef TEMPORALKEYFRAMESTRATEGY_HPP
#define TEMPORALKEYFRAMESTRATEGY_HPP

#include <chrono>
#include <memory>
#include "KeyframeStrategy.hpp"

class TimeTracker{
    public:

    std::chrono::system_clock::time_point getKeyframeTimestamp(){
        return keyframeTimestamp;
    };

    void setKeyframeTimestamp(){
        keyframeTimestamp = std::chrono::high_resolution_clock::now();
    };

    private:
    std::chrono::system_clock::time_point keyframeTimestamp;
};

class TemporalKeyframeStrategy : public AbstractKeyframeStrategy{
    public:
    TemporalKeyframeStrategy(std::chrono::seconds threshold) : time_tracker(std::make_unique<TimeTracker>()), threshold(threshold){};

    bool isKeyframeInvalid(){
        if (std::chrono::high_resolution_clock::now() - time_tracker->getKeyframeTimestamp() > threshold){
            time_tracker->setKeyframeTimestamp();
            return true;
        }
        return false;
    }

    private:
    std::unique_ptr<TimeTracker> time_tracker;
    const std::chrono::seconds threshold;

};

#endif
