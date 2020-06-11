/**
 * Parse KITTI to simulate the stereo cam and imu data
 *
    struct ImuFrame {
        unsigned int timestamp_ms;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        Eigen::Vector4d quaternion;
    };

    struct StereoFrame {
        unsigned int timestamp_ms;
        cv::Mat frame_0;
        cv::Mat frame_1;
    };

    struct GpsFrame{

    };

    struct OutFrame{

    };

    void vio_init();
    void vio_deinit();

    void vio_setoutputcallback(std::function<void(std::shared_ptr<OutFrame>)>func); 
    void vio_addimage(std::shared_ptr<StereoFrame> frame); void
    vio_addimu(std::shared_ptr<ImuFrame> imu); void
    vio_addgps(std::shared_ptr<GpsFrame> gps);
 *
 * backed/sample
 *
    int main(){
        vio_init();
        vio_setoutputcallback([&](std::shared_ptr<OutFrame> output){
            // process vio output
        });

        while(flag_exit)){
            if(frame_ready){
                vio_addimage(image);
            }
            if(imu_ready){
                vio_addimu(imu);
            }
            if(gps_ready){
                vio_addgps(gps);
            }
        }

        vio_deinit();
    }

 *
 *
 *
 *
 */

#ifndef VIO_TEST_KITTI_LOADER_H_
#define VIO_TEST_KITTI_LOADER_H_

#include <string>
#include <vector>

#include "DatasetLoader.h"

namespace msckf_vio {

// #define KITTI_DATASET_PATH \
//     "Datasets/KITTI/raw_data/2011_09_26/2011_09_26_drive_0001_extract/"

class KittiLoader : public DatasetLoader {
   public:
    KittiLoader(std::string dataset_path);
    ~KittiLoader(){};

   private:
    void GetImagePathLists(std::vector<std::string>& image0_path,
                           std::vector<std::string>& image1_path,
                           std::vector<unsigned int> timestamp_ms);
    
    void GetImuPathLists(std::vector<std::string>& imu_path,
                         std::vector<unsigned int> timestamp_ms);

   private:
};

}  // namespace msckf_vio

#endif