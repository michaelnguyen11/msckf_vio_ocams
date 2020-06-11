#ifndef VIEWER_H_
#define VIEWER_H_

#include "pangolin/pangolin.h"
#include <glog/logging.h>

#include "math_utils.hpp"
#include "msckf_vio_state.h"

namespace msckf_vio {

class Viewer {
public:
    Viewer();
    ~Viewer();

    void updateView(const StateServer& vio_state);

    // display the final result when program's gonna quit
    void showMap();

private:
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState _s_cam;
    // Add named OpenGL viewports to window and provide 3D Handler
    pangolin::View _d_cam;

    // store key frame poses (i.e. image frame pose)
	std::vector<pangolin::OpenGlMatrix> _im_frame_poses;

    // scale the display map
    // TODO: add to app config
    const double _display_scale;

    // camera follow
    std::unique_ptr<pangolin::Var<bool>> _menuFollowCamera;
    bool _bFollow;

};

}  // namespace msckf_vio

#endif  // VIEWER_H_