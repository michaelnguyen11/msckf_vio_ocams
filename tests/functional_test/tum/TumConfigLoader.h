#ifndef VIO_TEST_CONFIG_H_
#define VIO_TEST_CONFIG_H_

#include <string>
#include "opencv2/core/mat.hpp"
#include "opencv2/core/persistence.hpp"

#include "functional_test/Common.h"

namespace msckf_vio {

class TumConfigLoader {
public:
    TumConfigLoader() = delete;
    TumConfigLoader(std::string config_path);

public:
    const CamConfig& getCam0Config() const;
    const CamConfig& getCam1Config() const;
    const ImuConfig& getImuConfig() const;

private:
    const std::string cam_config_path_, imu_config_path_;
    CamConfig cam0_config_, cam1_config_;
    ImuConfig imu_config_;

private:
    void readConfig();
    // read cam config, provide node name for the cam (e.g. "cam0")
    void readCamConfig(cv::FileStorage& fs, CamConfig& cam_config, std::string node_name);
    void readImuConfig(cv::FileStorage& fs, ImuConfig& imu_config);

};

} // namespace msckf_vio


#endif