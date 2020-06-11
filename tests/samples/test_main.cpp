/*
#include <iostream>
#include "DatasetLoader.h"
#include "kitti/KittiLoader.h"
#include "tum/TumLoader.h"
#include "opencv2/highgui.hpp"

using namespace dns;
using namespace vio;

int main(int argc, char* argv[]) {
    std::string dataset_path = "/Datasets/TUM_VI/dataset-outdoors4_512_16/";
    // DatasetLoader* kitti_loader = new KittiLoader(database_path);
    DatasetLoader* tum_loader = new TumLoader(dataset_path);

    while (true) {
        StereoData stereo_frame;
        tum_loader->grabStereoFrame(stereo_frame);

        cv::imshow("test_cam", stereo_frame.frame_0);
        if (cv::waitKey(20) == 'q')
            break;
    }
}
*/