#ifndef VIO_TEST_COMMON_H_
#define VIO_TEST_COMMON_H_

#include <eigen3/Eigen/Dense>
#include "opencv2/core/mat.hpp"

namespace msckf_vio {

struct CamConfig {
    cv::Mat T_cam_imu; // takes a vector from the IMU frame to the camx frame.
    cv::Mat T_cn_cnm1; // takes a vector from the cam0 frame to the cam1 frame. Only availabe in cam1_config
    std::vector<uchar> cam_overlaps; // (int)cam_overlaps[0]
    std::string cam_model;
    cv::Vec4d distortion_coeffs;
    std::string distortion_model;
    cv::Vec4d intrinsics;
    cv::Size2d resolution;
    std::string rostopic;
};

struct ImuConfig {
    std::string rostopic;
    double update_rate;
    double accelerometer_noise_density;
    double accelerometer_random_walk;
    double gyroscope_noise_density;
    double gyroscope_random_walk;
};

} // namespace msckf_vio

#endif