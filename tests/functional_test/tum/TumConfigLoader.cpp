#include "tum/TumConfigLoader.h"

#include <iostream>
#include <opencv2/core/persistence.hpp>

#include "utils/Utils.hpp"

namespace msckf_vio {

TumConfigLoader::TumConfigLoader(std::string dataset_path)
    : cam_config_path_(dataset_path + "/dso/camchain_cv.yaml"),
      imu_config_path_(dataset_path + "/dso/imu_config_cv.yaml") {

    // check if path exists
    if (!fs::fileExist(cam_config_path_)) {
        std::cout << "Cam config file does not exist" << std::endl;
        return;
    }
    if (!fs::fileExist(imu_config_path_)) {
        std::cout << "IMU config file does not exist" << std::endl;
        return;
    }

    // read config files
    readConfig();
}

const CamConfig& TumConfigLoader::getCam0Config() const {
    return cam0_config_;
}

const CamConfig& TumConfigLoader::getCam1Config() const {
    return cam1_config_;
}

const ImuConfig& TumConfigLoader::getImuConfig() const {
    return imu_config_;
}

void TumConfigLoader::readConfig() {
    cv::FileStorage fs(cam_config_path_, cv::FileStorage::READ);

    // read cam 0 config
    std::string cam0_node = "cam0";
    readCamConfig(fs, cam0_config_, cam0_node);

    // read cam 1 configs
    std::string cam1_node = "cam1";
    readCamConfig(fs, cam1_config_, cam1_node);

    fs.release();

    // Read IMU config
    cv::FileStorage fs1(imu_config_path_, cv::FileStorage::READ);
    readImuConfig(fs1, imu_config_);
}

void TumConfigLoader::readCamConfig(cv::FileStorage& fs, CamConfig& cam_config,
                                    std::string cam_node) {
    fs[cam_node]["T_cam_imu"] >> cam_config.T_cam_imu;
    fs[cam_node]["T_cn_cnm1"] >> cam_config.T_cn_cnm1;
    fs[cam_node]["cam_overlaps"] >> cam_config.cam_overlaps;
    fs[cam_node]["camera_model"] >> cam_config.cam_model;
    fs[cam_node]["distortion_coeffs"] >> cam_config.distortion_coeffs;
    fs[cam_node]["intrinsics"] >> cam_config.intrinsics;
    fs[cam_node]["resolution"] >> cam_config.resolution;
    fs[cam_node]["rostopic"] >> cam_config.rostopic;
}

void TumConfigLoader::readImuConfig(cv::FileStorage& fs, ImuConfig& imu_config) {
    fs["rostopic"] >> imu_config.rostopic;
    fs["update_rate"] >> imu_config.update_rate;
    fs["accelerometer_noise_density"] >> imu_config.accelerometer_noise_density;
    fs["accelerometer_random_walk"] >> imu_config.accelerometer_random_walk;
    fs["gyroscope_noise_density"] >> imu_config.gyroscope_noise_density;
    fs["gyroscope_random_walk"] >> imu_config.gyroscope_random_walk;
}

}  // namespace msckf_vio
