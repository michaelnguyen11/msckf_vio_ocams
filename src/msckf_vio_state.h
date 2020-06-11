#ifndef MSCKF_VIO_STATE_H
#define MSCKF_VIO_STATE_H

#include "imu_state.h"
#include "cam_state.h"

namespace msckf_vio {

/*
    * @brief StateServer Store one IMU states and several
    *    camera states for constructing measurement
    *    model.
    */
struct StateServer {
    IMUState imu_state;
    CamStateServer cam_states;

    // State covariance matrix
    Eigen::MatrixXd state_cov;
    Eigen::Matrix<double, 12, 12> continuous_noise_cov;
};

} // namespace msckf_vio

#endif