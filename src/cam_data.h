#ifndef STEREO_CAMERA_DATA_H_
#define STEREO_CAMERA_DATA_H_

#include <memory>
#include <mutex>
#include <condition_variable>
#include <queue>

#include "opencv2/core.hpp"

namespace msckf_vio
{

struct StereoData
{
    uint64_t timestamp_ms;
    cv::Mat frame_0; // left camera
    cv::Mat frame_1; // right camera
};

class StereoCameraFIFO
{
public:
    StereoCameraFIFO() = delete;
    explicit StereoCameraFIFO(const size_t &maxSize) : _maxSize(maxSize) {}
    ~StereoCameraFIFO() {};

    void push(const StereoData &data)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_queueFrame.size() < this->_maxSize)
        {
            _queueFrame.push(data);
        }
    }

    bool pop(StereoData &data)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_queueFrame.empty())
        {
            data = _queueFrame.front();
            _queueFrame.pop();
            return true;
        }
        else
        {
            return false;
        }
    }

    void resize(const size_t &size)
    {
        this->clear();
        std::lock_guard<std::mutex> lock(_mutex);
        _maxSize = size;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        std::queue<StereoData> empty;
        _queueFrame.swap(empty);
    }

    size_t size()
    {
        return _queueFrame.size();
    }

private:
    size_t _maxSize;

    std::queue<StereoData> _queueFrame;

    std::mutex _mutex;
};

} // namespace msckf_vio

#endif