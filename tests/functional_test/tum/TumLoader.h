#ifndef VIO_TEST_TUM_LOADER_H_
#define VIO_TEST_TUM_LOADER_H_

#include <string>
#include <memory>

#include "DatasetLoader.h"
#include "tum/TumConfigLoader.h"

namespace msckf_vio {

class TumLoader : public DatasetLoader {
public:
    TumLoader(std::string dataset_path);

public:
    bool grabStereoFrame(StereoData& stereo_frame);
    bool grabImuFrame(ImuData& imu_frame);

    const CamConfig& getCam0Config() const;
    const CamConfig& getCam1Config() const;
    const ImuConfig& getImuConfig() const;

private:
    // collect all image paths
    bool getImagePathLists();
    
    // load all IMU data to memory for fast reading
    bool getImuData();

private:
    std::shared_ptr<TumConfigLoader> tum_config_loader_;

    const std::string cam0_data_csv_path_, cam1_data_csv_path_, imu_data_path_;

    // image and timestamps
    std::vector<std::string> image0_names_, image1_names_;
    std::vector<uint64_t> cam0_ts_ms_, cam1_ts_ms_;

    // load all imu frames
    std::vector<ImuData> imu_frames_;

    // current reading indexes
    unsigned int curr_stereo_i_, curr_imu_i_;

    // prefix for image path
    std::string im0_path_prefix_, im1_path_prefix_;

};

}  // namespace msckf_vio

#endif