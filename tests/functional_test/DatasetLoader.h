#ifndef VIO_TEST_DATASET_LOADER_H_
#define VIO_TEST_DATASET_LOADER_H_

#include <string>

#include "Common.h"
#include "imu_data.h"
#include "cam_data.h"

namespace msckf_vio {

class DatasetLoader {
public:
    DatasetLoader() = delete;
    DatasetLoader(std::string dataset_path):dataset_path(dataset_path){};
    ~DatasetLoader(){};

public:
    virtual bool grabStereoFrame(StereoData& stereo_frame){};
    virtual bool grabImuFrame(ImuData& imu_frame){};

    // virtual const CamConfig& getCam0Config() const {};
    // virtual const CamConfig& getCam1Config() const {};
    // virtual const ImuConfig& getImuConfig() const {};

protected:
    const std::string dataset_path;

};

}  // namespace msckf_vio

#endif