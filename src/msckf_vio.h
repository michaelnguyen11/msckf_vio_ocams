/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_H
#define MSCKF_VIO_H

#include <map>
#include <set>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include "imu_state.h"
#include "cam_state.h"
#include "feature.hpp"

#include "cam_data.h"
#include "imu_data.h"
#include "image_processor_output.h"
#include "msckf_vio_state.h"

namespace msckf_vio {
/*
 * @brief MsckfVio Implements the algorithm in
 *    Anatasios I. Mourikis, and Stergios I. Roumeliotis,
 *    "A Multi-State Constraint Kalman Filter for Vision-aided
 *    Inertial Navigation",
 *    http://www.ee.ucr.edu/~mourikis/tech_reports/TR_MSCKF.pdf
 */
class MsckfVio {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    MsckfVio();
    // Disable copy and assign constructor
    MsckfVio(const MsckfVio&) = delete;
    MsckfVio operator=(const MsckfVio&) = delete;

    // Destructor
    ~MsckfVio() {}

    /*
     * @brief initialize Initialize the VIO.
     */
    bool initialize();

    StateServer start(const ImageProcessorOutput::Ptr &imgProcInput, ImuBuffer &imuBuffer);

    typedef boost::shared_ptr<MsckfVio> Ptr;
    typedef boost::shared_ptr<const MsckfVio> ConstPtr;

  private:
    /*
     * @brief loadParameters
     *    Load parameters from the parameter server.
     */
    bool loadParameters();

    /*
     * @brief publish Publish the results of VIO.
     * @param time The time stamp of output msgs.
     */
    void publish();

    /*
     * @brief initializegravityAndBias
     *    Initialize the IMU bias and initial orientation
     *    based on the first few IMU readings.
     */
    void initializeGravityAndBias(const ImuBuffer &imuBuffer);

    // Filter related functions
    // Propogate the state
    void batchImuProcessing(const double& time_bound, ImuBuffer &imuBuffer);
    void processModel(const double& time,
        const Eigen::Vector3d& m_gyro,
        const Eigen::Vector3d& m_acc);
    void predictNewState(const double& dt,
        const Eigen::Vector3d& gyro,
        const Eigen::Vector3d& acc);

    // Measurement update
    void stateAugmentation(const double& time);
    void addFeatureObservations(const ImageProcessorOutput::Ptr &imgProcInput);
    // This function is used to compute the measurement Jacobian
    // for a single feature observed at a single camera frame.
    void measurementJacobian(const StateIDType& cam_state_id,
        const FeatureIDType& feature_id,
        Eigen::Matrix<double, 4, 6>& H_x,
        Eigen::Matrix<double, 4, 3>& H_f,
        Eigen::Vector4d& r);
    // This function computes the Jacobian of all measurements viewed
    // in the given camera states of this feature.
    void featureJacobian(const FeatureIDType& feature_id,
        const std::vector<StateIDType>& cam_state_ids,
        Eigen::MatrixXd& H_x, Eigen::VectorXd& r);
    void measurementUpdate(const Eigen::MatrixXd& H,
        const Eigen::VectorXd& r);
    bool gatingTest(const Eigen::MatrixXd& H,
        const Eigen::VectorXd&r, const int& dof);
    void removeLostFeatures();
    void findRedundantCamStates(
        std::vector<StateIDType>& rm_cam_state_ids);
    void pruneCamStateBuffer();
    // Reset the system online if the uncertainty is too large.
    void onlineReset();

    // Chi squared test table.
    static std::map<int, double> chi_squared_test_table;

    // State vector
    StateServer state_server;
    // Maximum number of camera states
    int max_cam_state_size;

    // Features used
    MapServer map_server;

    // Indicate if the gravity vector is set.
    bool is_gravity_set;

    // Indicate if the received image is the first one. The
    // system will start after receiving the first image.
    bool is_first_img;

    // The position uncertainty threshold is used to determine
    // when to reset the system online. Otherwise, the ever-
    // increaseing uncertainty will make the estimation unstable.
    // Note this online reset will be some dead-reckoning.
    // Set this threshold to nonpositive to disable online reset.
    double position_std_threshold;

    // Tracking rate
    double tracking_rate;

    // Threshold for determine keyframes
    double translation_threshold;
    double rotation_threshold;
    double tracking_rate_threshold;
};

typedef MsckfVio::Ptr MsckfVioPtr;
typedef MsckfVio::ConstPtr MsckfVioConstPtr;

} // namespace msckf_vio

#endif
