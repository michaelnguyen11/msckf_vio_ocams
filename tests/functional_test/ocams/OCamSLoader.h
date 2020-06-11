#ifndef VIO_TEST_OCAMS_LOADER_H_
#define VIO_TEST_OCAMS_LOADER_H_

#include "DatasetLoader.h"

namespace msckf_vio {

class OCamSLoader : public DatasetLoader {
public:
    OCamSLoader() = delete;
    OCamSLoader(std::string dataset_path);

public:
    bool grabStereoFrame(StereoData& stereo_frame);
    bool grabImuFrame(ImuData& imu_frame);

private:
    // collect all image paths
    bool getImagePathLists();
    
    // load all IMU data to memory for fast reading
    bool getImuData();

private:
    const std::string imu_data_path_;

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