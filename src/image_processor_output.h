#ifndef IMAGE_PROCESSOR_OUTPUT_H_
#define IMAGE_PROCESSOR_OUTPUT_H_

#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>

namespace msckf_vio
{

// Measurement for one features
class FeatureMeasurement
{
public:
    FeatureMeasurement() : id(0),
                           u0(0.0),
                           v0(0.0),
                           u1(0.0),
                           v1(0.0) {}
    ~FeatureMeasurement() {}

    unsigned long long int id;
    // Normalized feature coordinates (with identity intrinsic matrix)
    // horizontal and vertical coordinate of cam0
    double u0;
    double v0;
    // horizontal and vertical coordinate of cam1
    double u1;
    double v1;
};

// Measurements for features in stereo camera frame
class ImageProcessorOutput
{
public:
    typedef std::shared_ptr<ImageProcessorOutput> Ptr;

    uint64_t stereoCamCurrTimestamp_ms;
    // All features on the current image,
    // including tracked ones and newly detected ones.
    std::vector<FeatureMeasurement> features;
};

} // namespace msckf_vio

#endif // !IMAGE_PROCESSOR_OUTPUT_H_
