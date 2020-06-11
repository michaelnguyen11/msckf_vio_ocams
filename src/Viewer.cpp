#include "Viewer.h"

#include <chrono>
#include <thread>

#include "Visualize.hpp"

namespace msckf_vio {

using namespace std::chrono_literals;
Viewer::Viewer()
    : _s_cam(
          pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
          pangolin::ModelViewLookAt(0, 3.5, 9, 0, 0, 0, 0, -1, 0)),
    _display_scale(1000) {
    LOG(INFO) << "Viewer started";
    pangolin::CreateWindowAndBind("DNS Viewer", 1024, 768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    _s_cam = pangolin::OpenGlRenderState(
          pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
          pangolin::ModelViewLookAt(0, 3.5, 9, 0, 0, 0, 0, -1, 0));

    // create interactive windows
    _d_cam = pangolin::CreateDisplay()
                 .SetBounds(0.0, 1.0, pangolin::Attach::Pix(0), 1.0,
                            -1024.0f / 768.0f)
                 .SetHandler(new pangolin::Handler3D(_s_cam));

	pangolin::CreatePanel("menu").SetBounds(0.95,1.0,0.0,pangolin::Attach::Pix(175));
    _menuFollowCamera = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu. Follow Camera", true, true));
    _bFollow = true;
}

void Viewer::updateView(const StateServer& vio_state) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::OpenGlMatrix Tbw_pgl;

    // Convert the IMU frame to the body frame.
    const IMUState& imu_state = vio_state.imu_state;
    Eigen::Isometry3d T_i_w = Eigen::Isometry3d::Identity();
    T_i_w.linear() = quaternionToRotation(imu_state.orientation).transpose();
    T_i_w.translation() = imu_state.position / _display_scale;
    Eigen::Isometry3d T_b_w =
        imu_state.T_imu_body * T_i_w * imu_state.T_imu_body.inverse();

    GetCurrentOpenGLPoseMatrix(Tbw_pgl, T_b_w);

    // project the cam whether to follow the view angle
    if (*_menuFollowCamera && _bFollow) {
        _s_cam.Follow(Tbw_pgl);
    }
    else if (*_menuFollowCamera && !_bFollow) {
        _s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,3.5,9, 0,0,0, 0,-1,0));
        _s_cam.Follow(Tbw_pgl);
        _bFollow = true;
    }
    else if (!*_menuFollowCamera && _bFollow)
        _bFollow = false;


    _d_cam.Activate(_s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    DrawCurrentPose(Tbw_pgl);
    DrawKeyFrames(_im_frame_poses);

    _im_frame_poses.push_back(Tbw_pgl);
    pangolin::FinishFrame();
}

void Viewer::showMap() {
    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        _d_cam.Activate(_s_cam);

        DrawKeyFrames(_im_frame_poses);

        // Swap frames and Process Events
        pangolin::FinishFrame();

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

Viewer::~Viewer() {}

}  // namespace msckf_vio