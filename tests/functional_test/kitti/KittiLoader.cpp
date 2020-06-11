#include <iostream>

#include "Common.h"
#include "kitti/KittiLoader.h"

namespace msckf_vio {

KittiLoader::KittiLoader(std::string dataset_path): DatasetLoader(dataset_path) {
    // TODO: load paths of stereo images and imu, 
    // imu data could be loaded directly to memory to save read / write time
    std::cout << dataset_path << std::endl;
}


}  // namespace msckf_vio
