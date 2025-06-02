#pragma once
#include <deque>
#include <numeric>
#include <stddef.h>

class RollingMeanAccumulator
{
public:
    explicit RollingMeanAccumulator(size_t window_size)
        : window_size_(window_size) {}

    void accumulate(double value)
    {
        if (values_.size() >= window_size_)
        {
            values_.pop_front();
        }
        values_.push_back(value);
    }

    double getRollingMean() const
    {
        if (values_.empty())
            return 0.0;
        return std::accumulate(values_.begin(), values_.end(), 0.0) / values_.size();
    }

    void reset()
    {
        values_.clear();
    }

private:
    std::deque<double> values_;
    size_t window_size_;
};
