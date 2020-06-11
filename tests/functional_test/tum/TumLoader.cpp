#include "tum/TumLoader.h"

#include <fstream>
#include <iostream>

#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "utils/Utils.hpp"

namespace msckf_vio {

TumLoader::TumLoader(std::string dataset_path)
    : DatasetLoader(dataset_path),
      cam0_data_csv_path_(dataset_path + "/mav0/cam0/data.csv"),
      cam1_data_csv_path_(dataset_path + "/mav0/cam1/data.csv"),
      imu_data_path_(dataset_path + "/mav0/imu0/data.csv") {

    // load cameras and imu configs
    // tum_config_loader_ = std::make_shared<TumConfigLoader>(dataset_path);

    // load image paths and timestamps
    getImagePathLists();

    // load imu frame data
    getImuData();

    im0_path_prefix_ = dataset_path + "/mav0/cam0/data/";
    im1_path_prefix_ = dataset_path + "/mav0/cam1/data/";

    curr_stereo_i_ = 0;
    curr_imu_i_ = 0;
}

bool TumLoader::grabStereoFrame(StereoData& stereo_frame) {
    if (curr_stereo_i_ < cam0_ts_ms_.size()) {
        stereo_frame.timestamp_ms = cam0_ts_ms_[curr_stereo_i_];

        // read grayscale image from image path
        stereo_frame.frame_0 =
            cv::imread(im0_path_prefix_ + image0_names_[curr_stereo_i_],
                       cv::IMREAD_GRAYSCALE);
        stereo_frame.frame_1 =
            cv::imread(im1_path_prefix_ + image1_names_[curr_stereo_i_],
                       cv::IMREAD_GRAYSCALE);

        curr_stereo_i_++;

    } else {
        return false;
    }

    return true;
}

bool TumLoader::grabImuFrame(ImuData& imu_frame) {
    if (curr_imu_i_ < imu_frames_.size()) {
        imu_frame = imu_frames_[curr_imu_i_];
        curr_imu_i_++;
    } else {
        return false;
    }

    return true;
}

const CamConfig& TumLoader::getCam0Config() const {
    return tum_config_loader_->getCam0Config();
}

const CamConfig& TumLoader::getCam1Config() const {
    return tum_config_loader_->getCam1Config();
}

const ImuConfig& TumLoader::getImuConfig() const {
    return tum_config_loader_->getImuConfig();
}

bool TumLoader::getImagePathLists() {
    std::ifstream infile0(cam0_data_csv_path_), infile1(cam1_data_csv_path_);

    std::string f0_line, f1_line;
    // skip the first line
    std::getline(infile0, f0_line);
    std::getline(infile1, f1_line);
    while (std::getline(infile0, f0_line) && std::getline(infile1, f1_line)) {
        // split the strings
        // a line includes: <timestamp_ns>,<image_name>
        std::vector<std::string> l_str0, l_str1;
        str::split(l_str0, f0_line, ",");
        str::split(l_str1, f1_line, ",");

        // push to vectors
        // convert timestamp from nanosecond to milisecond
        cam0_ts_ms_.push_back(std::lrint(std::stol(l_str0[0]) / 1000000.0));
        image0_names_.push_back(l_str0[1]);
        cam1_ts_ms_.push_back(std::lrint(std::stol(l_str1[0]) / 1000000.0));
        image1_names_.push_back(l_str1[1]);
    }

    infile0.close();
    infile1.close();

    // std::cout << cam0_ts_ms_.size() << " " << image1_names_.size() << std::endl;
}

bool TumLoader::getImuData() {

    std::ifstream infile(imu_data_path_);
    std::string line;
    // skip the first line
    std::getline(infile, line);
    while (std::getline(infile, line)) {
        // split the line
        std::vector<std::string> l_str;
        str::split(l_str, line, ",");

        ImuData tmp_frame;
        // convert from nanosecond to milisecond
        tmp_frame.timestamp_ms = std::lrint(std::stol(l_str[0]) / 1000000.0);

        // set value to eigen vectors
        tmp_frame.angularVelocity << std::stold(l_str[1]), std::stold(l_str[2]),
            std::stold(l_str[3]);

        tmp_frame.linearAcceleration << std::stold(l_str[4]), std::stold(l_str[5]),
            std::stold(l_str[6]);

        imu_frames_.push_back(tmp_frame);

        // increase the cout precision to see more
        // std::cout << tmp_frame.accel << std::endl;
    }
}

}  // namespace msckf_vio