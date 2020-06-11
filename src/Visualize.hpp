/*
 * @Descripttion: Modified after Xiaochen Qiu (LARVIO)
 */

#ifndef VISUALIZE_HPP_
#define VISUALIZE_HPP_

#include <pangolin/pangolin.h>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <opencv2/opencv.hpp>


namespace msckf_vio {

static void DrawCurrentPose(pangolin::OpenGlMatrix &Twc) {
    const float &w = 0.08; // 0.08
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif
    glLineWidth(3);
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

static void DrawKeyFrames(const std::vector<pangolin::OpenGlMatrix>& poses) {
    const float &w = 0.05; // 0.05
    const float h = w*0.75;
    const float z = w*0.6;

    for(auto pose : poses) {
        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(pose.m);
#else
        glMultMatrixd(pose.m);
#endif

        glLineWidth(1);
        glColor3f(1.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

static void GetCurrentOpenGLPoseMatrix(pangolin::OpenGlMatrix &M, const Eigen::Isometry3d &Tbw) {
    Eigen::Matrix3d R = Tbw.rotation();
    Eigen::Vector3d t = Tbw.translation();

    M.m[0] = R(0,0);
    M.m[1] = R(1,0);
    M.m[2] = R(2,0);
    M.m[3]  = 0.0;

    M.m[4] = R(0,1);
    M.m[5] = R(1,1);
    M.m[6] = R(2,1);
    M.m[7]  = 0.0;

    M.m[8] = R(0,2);
    M.m[9] = R(1,2);
    M.m[10] = R(2,2);
    M.m[11]  = 0.0;

    M.m[12] = t(0);
    M.m[13] = t(1);
    M.m[14] = t(2);
    M.m[15]  = 1.0;
}

// Keypoint visualisation
static void plotKeyPoint(const cv::Mat& im, const std::vector<cv::KeyPoint>& kpts) {
    cv::Mat out_im;
    cv::drawKeypoints(im, kpts, out_im);

    cv::imshow("Current keypoints", out_im);
    if (cv::waitKey(1) == 'q') return;
}

}

#endif // VISUALIZE_HPP_