#include <chrono>
#include <thread>
#include <iostream>
#include <signal.h>
#include <glog/logging.h>

#include "DatasetLoader.h"
#include "tum/TumLoader.h"
#include "ocams/OCamSLoader.h"
#include "Viewer.h"

#include "image_processor.h"
#include "msckf_vio.h"
#include "utils/test_vio_visualization.hpp"

using namespace msckf_vio;

StereoCameraFIFO stereoQueue(1000);
ImuBuffer imuBuffer;
ImuBuffer imuBufferProcessed;

static bool is_streaming = true;
static void sig_handler(int sig)
{
    LOG(INFO) << std::endl << "VIO stop by user";
    is_streaming = false;
}

void stereoReadRun(const std::shared_ptr<DatasetLoader>& data_loader_ptr) {
    while (is_streaming) {
        StereoData stereo_frame;
        if (data_loader_ptr->grabStereoFrame(stereo_frame))
        {
            stereoQueue.push(stereo_frame);
        }
    }
}

void imuReadRun(const std::shared_ptr<DatasetLoader>& data_loader_ptr) {
    while (is_streaming) {
        ImuData imu_frame;
        if (data_loader_ptr->grabImuFrame(imu_frame))
        {
            imuBuffer.push_back(imu_frame);
        }
    }
}

int main(int argc, char const *argv[])
{
    const char *params = "{ help           | false  | print usage          }"
                         "{ path           |        | (string) path/to/dataset }";

    cv::CommandLineParser config(argc, argv, params);
    if (config.get<bool>("help"))
    {
        config.printMessage();
        exit(0);
    }

    std::string dataset_path = config.get<std::string>("path");
    if (dataset_path.empty())
    {
        LOG(ERROR) << "dataset_path is empty";
        config.printMessage();
        exit(0);
    }

    // Define google logging
    FLAGS_alsologtostderr = 1;
    FLAGS_minloglevel = 0;
    FLAGS_v = 2;
    FLAGS_max_log_size = 100;
    google::SetLogDestination(0, "./msckf_vio_ocams");
    google::InitGoogleLogging(argv[0]);

    // std::string dataset_path = "/mnt/6e1ef38e-db2f-4eda-ad11-31252df3b87b/data/Datasets/TUM_VI/dataset-outdoors4_512_16";
    // std::shared_ptr<DatasetLoader> dataset_loader_ptr = std::make_shared<TumLoader>(dataset_path);
    std::shared_ptr<DatasetLoader> dataset_loader_ptr = std::make_shared<OCamSLoader>(dataset_path);

    std::shared_ptr<Viewer> viewer_ptr = std::make_shared<Viewer>();

    // handle signal by user
    struct sigaction act;
    act.sa_handler = sig_handler;
    sigaction(SIGINT, &act, NULL);

    std::thread stereo_read_thread (stereoReadRun, std::ref(dataset_loader_ptr));
    std::thread imu_read_thread (imuReadRun, std::ref(dataset_loader_ptr));

    // =================== VIO process
    // image processor
    std::shared_ptr<ImageProcessor> image_processor_ptr = std::make_shared<ImageProcessor>();
    StateServer vioState;
    std::shared_ptr<MsckfVio> msckfVio = std::make_shared<MsckfVio>();

    while (is_streaming) {
        // vio_vis::stopwatch_start();

        ImageProcessorOutput::Ptr im_proc_output = std::make_shared<ImageProcessorOutput>();
        StereoData curr_frame;
        if (!stereoQueue.pop(curr_frame))
        {
            LOG(INFO) << "waiting for new frame...";
            usleep(10000);
            continue;
        }

        // LOG(INFO) << "Image processing: ";
        image_processor_ptr->start(curr_frame, imuBuffer, im_proc_output);
        // LOG(INFO) << "Output features: " << im_proc_output->features.size();
        // vio_vis::averagingFeatureMeasurement(im_proc_output->features);

        vioState = msckfVio->start(im_proc_output, imuBuffer);
        // vio_vis::printVioOutput(vioState);

        // vio_vis::stopwatch_stop();

        viewer_ptr->updateView(vioState);
    }

    stereo_read_thread.join();
    imu_read_thread.join();

    cv::destroyAllWindows();

    viewer_ptr->showMap();
    // viewer_ptr->stop();

    return 0;
}
