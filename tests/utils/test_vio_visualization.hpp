#include <iostream>
#include <fstream>
#include <glog/logging.h>
#include <chrono>

#include "eigen3/Eigen/Core"
#include "image_processor_output.h"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "msckf_vio_state.h"

using namespace msckf_vio;

namespace vio_vis {
static void averagingFeatureMeasurement(
    std::vector<FeatureMeasurement> features) {
    Eigen::Vector4d sum;
    for (const auto feat : features) {
        Eigen::Vector4d feat_vec{feat.u0, feat.v0, feat.u1, feat.v1};
        sum += feat_vec;
    }
    // show sum
    sum /= features.size();
    VLOG(1) << "Average of " << features.size() << " feature measurements: "
              << "u0: " << sum.x() << " | v0: " << sum.y()
              << " | u1: " << sum.z() << " | v1: " << sum.w();
}

static void plotKeyPoint(cv::Mat im, std::vector<cv::KeyPoint> kpts) {
    cv::Mat out_im = im;
    cv::drawKeypoints(im, kpts, out_im);

    cv::imshow("test", out_im);
    if (cv::waitKey(1) == 'q') return;
}

static void printVioOutput(StateServer vioState) {
    //--------------- IMU
    Eigen::Vector3d imu_pos = vioState.imu_state.position;
    VLOG(1) << "IMU position: x: " << imu_pos.x()
              << " | y: " << imu_pos.y()
              << " | z: " << imu_pos.z();
    Eigen::Vector3d imu_vel = vioState.imu_state.velocity;
    VLOG(1) << "IMU velocity: x: " << imu_vel.x()
              << " | y: " << imu_vel.y()
              << " | z: " << imu_vel.z();

    if (vioState.cam_states.size() > 0) {
        // number of states will be 0 after online reset
        Eigen::Vector3d camera_pos0 = vioState.cam_states.rbegin()->second.position;
        VLOG(1) << "Feature 0 position: x: " << camera_pos0.x()
                << " | y: " << camera_pos0.y()
                << " | z: " << camera_pos0.z();
    }
}

static std::chrono::steady_clock::time_point _stopwatch_start_ms;

static void stopwatch_start() {
    _stopwatch_start_ms = std::chrono::steady_clock::now();
}

static void stopwatch_stop() {
    int64_t time_since_start =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - _stopwatch_start_ms)
            .count();

    VLOG(1) << "Processing time: " << time_since_start << "ms";
}

// vector to string
// static void vec2str(const Eigen::VectorXd& vec, std::string& s) {
//     s = "";
//     for (auto x : vec)
//         s += std::to_string(x) + " ";
// }

// log all vio output to file for analysis
static void save(const ImageProcessorOutput::Ptr& improc_output_ptr, const ImuBuffer& imu_buffer) {
    // Averaging feature vectors
    std::vector<FeatureMeasurement> features = improc_output_ptr->features;
    Eigen::Vector4d sum;
    for (const auto feat : features) {
        Eigen::Vector4d feat_vec{feat.u0, feat.v0, feat.u1, feat.v1};
        sum += feat_vec;
    }
    // show sum
    sum /= features.size();

    // Cropping and averaging IMU data
    Eigen::Vector3d gyro_sum, accel_sum;
    uint64_t cutoff_time = improc_output_ptr->stereoCamCurrTimestamp_ms;
    int imu_len = 0;
    for (const auto &imuData : imu_buffer)
    {
        uint64_t imuTime = imuData.timestamp_ms;
        if (imuTime < cutoff_time) {
            gyro_sum += imuData.angularVelocity;
            accel_sum += imuData.linearAcceleration;
            ++imu_len;
        }
        else
            break;
    }

    if (imu_len > 0) {
        gyro_sum /= imu_len;
        accel_sum /= imu_len;
    }

    // Write to file
    std::ofstream data_file;
    data_file.open("../tests/data/output_data.txt", std::ios_base::app);
    // line structure: timestamp_ms - average feature vector - avg. imu gyro - avg. imu accelerometer
    // line len: 1 + 4 + 3 + 3 = 11
    std::string sum_str, gyro_sum_str, accel_sum_str;
    // vec2str(sum, sum_str);
    // vec2str(gyro_sum, gyro_sum_str);
    // vec2str(accel_sum, accel_sum_str);
    data_file << cutoff_time << " " << sum_str << gyro_sum_str << accel_sum_str << "\n";
}

}  // namespace vio_vis