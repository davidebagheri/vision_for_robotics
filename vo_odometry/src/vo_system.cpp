#include "vo_odometry/vo_system.h"
#include <algorithm>

VOSystem::VOSystem(const cv::FileStorage& params){
    // Get Params
    n_init_frame_ = (int)params["n_init_frame"];

    // Init variables
    state_ = FIRST_IMAGE;
    n_frame_ = -1;
    tracker_ = new Tracker(params);
    initializer_ = new Initializer(params);
    camera_ = new Camera(params);
}

bool VOSystem::processImage(const cv::Mat& image){
    n_frame_++;

    switch (state_)
    {
        case FIRST_IMAGE:
        {
            first_frame_ = new Frame(image);
            tracker_->extractKeypoints(first_frame_);
            first_frame_->setPose(cv::Matx33f::eye(), cv::Vec3f(0.0, 0.0, 0.0));   // Initial pose
            prev_frame_ = first_frame_;  
            current_frame_ = first_frame_; 

            state_ = INITIALIZING;
            break;
        }
        case INITIALIZING:
        {
            current_frame_ = new Frame(image);
            
            // Track previous keypoints
            tracker_->trackPoints(*prev_frame_, *current_frame_);

            // Propagate matches to the first frame if at least 3 frames have been processed
            if (first_frame_ != prev_frame_){
                for (int i = 0; i < first_frame_->keypoints_.size(); i++){
                    int first_frame_match_id = first_frame_->matches_[i];
                    if (first_frame_match_id >= 0){
                        int prev_frame_match_id = prev_frame_->matches_[first_frame_match_id];
                        if (prev_frame_match_id >= 0) {
                            first_frame_->matches_[i] = prev_frame_match_id;
                        } else {
                            first_frame_->matches_[i] = -1;
                        }
                    }
                }
            }

            if (n_frame_ == n_init_frame_){
                // Initialize through 5pt algorithm-RANSAC and triangulate inliers
                initializer_->initialize(current_frame_, first_frame_, *camera_);
                state_ = TRACKING;
            } 

            prev_frame_ = current_frame_;
            break;
        }
        case TRACKING:
        {

            break;
        }
        default:
        {
            std::cerr << "No possible state is set!" << std::endl;
            return false;
        }
    }
    return true;
}