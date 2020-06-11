#include <iostream>
#include <fstream>
#include <experimental/filesystem>

#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "utils/Utils.hpp"
#include "ocams/OCamSLoader.h"

namespace msckf_vio {

OCamSLoader::OCamSLoader(std::string dataset_path) : DatasetLoader(dataset_path), imu_data_path_(dataset_path + "/imu0.csv") {
    im0_path_prefix_ = dataset_path + "/cam0/";
    im1_path_prefix_ = dataset_path + "/cam1/";

    // load all image paths
    getImagePathLists();

    // load all IMU data
    getImuData();

    curr_stereo_i_ = 0;
    curr_imu_i_ = 0;
}

bool OCamSLoader::getImagePathLists() {
    // get all image 0 paths
    for (const auto & entry : std::experimental::filesystem::directory_iterator(im0_path_prefix_)) {
        image0_names_.push_back(entry.path());
    }
    // get all image 1 paths
    for (const auto & entry : std::experimental::filesystem::directory_iterator(im1_path_prefix_)) {
        image1_names_.push_back(entry.path());
    }

    // ensure that the numbers of image files in both cams are equal
    if (image0_names_.size() != image1_names_.size())
        LOG(ERROR) << "Numbers of images in both cams' folder are not equal";

    // sort all image
    std::sort(image0_names_.begin(), image0_names_.end());
    std::sort(image1_names_.begin(), image1_names_.end());

    // extract the timestamps from the two image lists
    for (std::size_t i = 0; i < image0_names_.size(); i++) {
        // get the file's base name
        auto i_ts = image0_names_[i].substr(image0_names_[i].find_last_of("/\\") + 1);
        // remove the extension and convert to uint64_t
        i_ts = i_ts.substr(0, i_ts.size()-4);

        // convert from nanosecond to milisecond and add to the list
        cam0_ts_ms_.push_back(std::stoull(i_ts) / 1000000);
    }

    // extract the timestamps from the two image lists
    for (std::size_t i = 0; i < image1_names_.size(); i++) {
        // get the file's base name
        auto i_ts = image1_names_[i].substr(image1_names_[i].find_last_of("/\\") + 1);
        // remove the extension and convert to uint64_t
        i_ts = i_ts.substr(0, i_ts.size()-4);

        // convert from nanosecond to milisecond and add to the list
        cam1_ts_ms_.push_back(std::stoull(i_ts) / 1000000);
    }

}

bool OCamSLoader::getImuData() {

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
    }
}

bool OCamSLoader::grabStereoFrame(StereoData& stereo_frame){
    if (curr_stereo_i_ < cam0_ts_ms_.size()) {
        stereo_frame.timestamp_ms = cam0_ts_ms_[curr_stereo_i_];

        // read grayscale image from image path
        stereo_frame.frame_0 =
            cv::imread(image0_names_[curr_stereo_i_],
                       cv::IMREAD_GRAYSCALE);
        stereo_frame.frame_1 =
            cv::imread(image1_names_[curr_stereo_i_],
                       cv::IMREAD_GRAYSCALE);

        curr_stereo_i_++;

    } else {
        return false;
    }

    return true;
}
bool OCamSLoader::grabImuFrame(ImuData& imu_frame){
    if (curr_imu_i_ < imu_frames_.size()) {
        imu_frame = imu_frames_[curr_imu_i_];
        curr_imu_i_++;
    } else {
        return false;
    }

    return true;
}

}