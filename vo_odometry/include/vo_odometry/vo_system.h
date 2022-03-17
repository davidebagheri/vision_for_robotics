#ifndef VO_SYSTEM_H
#define VO_SYSTEM_H

#include "vo_odometry/frame.h"
#include "vo_odometry/tracker.h"
#include "vo_odometry/initializer.h"
#include "vo_odometry/converter.h"
#include "vo_odometry/camera.h"

enum State {FIRST_IMAGE, INITIALIZING, TRACKING};

class VOSystem{
public:
    VOSystem(const cv::FileStorage& params);

    bool processImage(const cv::Mat& image);

    const State& getState();

    bool getPose(cv::Matx33f& R, cv::Vec3f& t);

    bool get3dPoints(std::vector<cv::Point3f>& points_3d);

    void visualize(cv::Mat img);

private:
    State state_;

    // Modules
    Tracker* tracker_;
    Initializer* initializer_;
    Camera* camera_;
    
    // Variables
    int n_frame_;
    Frame* first_frame_;
    Frame* prev_frame_;
    Frame* current_frame_;
    
    // Params
    int n_init_frame_;
};

#endif 