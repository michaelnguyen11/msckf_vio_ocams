#ifndef IMU_DATA_H_
#define IMU_DATA_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "SafeVector.h"

namespace msckf_vio
{

struct ImuData
{
    uint64_t timestamp_ms;
    Eigen::Vector3d angularVelocity;
    Eigen::Vector3d linearAcceleration;
    Eigen::Vector4d quaternion;
};

typedef SafeVector<ImuData> ImuBuffer;

} // namespace msckf_vio

#endif