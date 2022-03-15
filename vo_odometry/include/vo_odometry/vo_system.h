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

    State getState(){
        return state_;
    }

    bool getPose(cv::Matx33f& R, cv::Vec3f& t){
        if (current_frame_ != nullptr){
            R = current_frame_->R_;
            t = current_frame_->t_;
            return true;
        } 
        return false;
    }

    bool get3dPoints(std::vector<cv::Point3f>& points_3d){
        if (current_frame_ != nullptr and current_frame_->points_3d_.size() != 0){
            points_3d = current_frame_->points_3d_;
            return true;
        }
        return false;
    }

    void visualize(cv::Mat img){
        for (int i = 0; i < current_frame_->keypoints_.size(); i++){
                cv::circle(img, current_frame_->keypoints_[i], 2, cv::Scalar(0,255,0), 2);            
        }
    }

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