#ifndef VO_SYSTEM_H
#define VO_SYSTEM_H

#include "vo_odometry/frame.h"
#include "vo_odometry/tracker.h"
#include "vo_odometry/initializer.h"
#include "vo_odometry/map.h"
#include "vo_odometry/camera.h"

enum State {NONE, FIRST_IMAGE, INITIALIZING, TRACKING};

class VOSystem{
public:
    VOSystem(const cv::FileStorage& params);

    ~VOSystem(){
        delete tracker_;
        delete initializer_;
        delete camera_;
        delete map_;
    }

    bool processImage(const cv::Mat& image);

    void visualize(cv::Mat* img);

private:
    State state_, prev_state_;

    Tracker* tracker_;
    Initializer* initializer_;
    Camera* camera_;
    
    int n_frame_;
    Frame* first_frame_ = nullptr;
    Frame* prev_frame_ = nullptr;
    Frame* current_frame_ = nullptr;
    Map* map_ = nullptr;
        
    // Params
    int n_init_frame_;
};

#endif 