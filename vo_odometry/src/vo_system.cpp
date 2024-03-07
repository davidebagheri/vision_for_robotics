#include "vo_odometry/vo_system.h"
#include <algorithm>

VOSystem::VOSystem(const cv::FileStorage& params){
    // Get Params
    n_init_frame_ = (int)params["n_init_frame"];

    // Init variables
    state_ = FIRST_IMAGE;
    prev_state_ = NONE;

    n_frame_ = -1;
    tracker_ = new Tracker(params);
    initializer_ = new Initializer(params);
    camera_ = new Camera(params);
    map_ = new Map();
}

bool VOSystem::processImage(const cv::Mat& image){
    n_frame_++;

    std::cout << "********************** Frame " << n_frame_ << " **********************" << std::endl;

    // General update: frames and previous state
    if (prev_frame_ and prev_frame_->frame_n_ != 0) delete prev_frame_; // Delete the previous frame (but not if it is the first frame!)
    prev_frame_ = current_frame_;
    current_frame_ = new Frame(image, n_frame_);
    prev_state_ = state_;
    
    switch (state_)
    {
        case FIRST_IMAGE:
        {
            std::cout << "First image incoming" << std::endl;
            
            first_frame_ = current_frame_;
            first_frame_->keypoints_= tracker_->extractKeypoints(*first_frame_, 0, 0.001, false);
            first_frame_->pose = cv::Affine3f::Identity();   // Initial pose
            
            std::cout << "Extracted " << first_frame_->keypoints_.size() << " keypoints" << std::endl;

            state_ = INITIALIZING;
            break;
        }
        case INITIALIZING:
        {
            // Track previous keypoints
            tracker_->trackKeypoints(*prev_frame_, current_frame_);
            
            // Propagate matches to the first frame if at least 3 frames have been processed
            if (n_frame_ > 1){
                for (int i = 0; i < current_frame_->keypoints_.size(); i++){
                    int prev_frame_match = current_frame_->matches_[i];
                    current_frame_->matches_[i] = prev_frame_->matches_[prev_frame_match]; 
                }
            }

            if (n_frame_ == n_init_frame_){
                // Initialize through 5pt algorithm-RANSAC and triangulate inliers
                initializer_->initialize(current_frame_, first_frame_, map_, *camera_);
    
                state_ = TRACKING;
            }
                        
            break;
        }
        case TRACKING:
        {   
            // Track previous keypoints
            tracker_->trackKeypoints(*prev_frame_, current_frame_);

            // Extract new points 
            std::vector<cv::Point2f> new_kpts = tracker_->extractKeypoints(
                *current_frame_, 
                std::max<int>(1000 - current_frame_->keypoints_.size() - current_frame_->candidate_keypoints_.size(), 1), 
                0.01, 
                true);

            // Estimate pose through pnp
            tracker_->estimatePose(*prev_frame_, current_frame_, *map_, *camera_);

            // Triangulate new points
            tracker_->triangulateUnmatchedKeypoints(prev_frame_, current_frame_, map_, *camera_);

            // Detect new points to track
            tracker_->addNewKeypoints(current_frame_, new_kpts);

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

void VOSystem::visualize(cv::Mat* img){
    switch (prev_state_)
    {
    case FIRST_IMAGE:
    {
        cv::Scalar color(0, 255, 0);    // Green

        for (int i = 0; i < current_frame_->keypoints_.size(); i++)
            cv::circle(*img, current_frame_->keypoints_[i], 2, color, 2);
        
        break;
    }
    case INITIALIZING:
    {
        cv::Scalar color(0, 255, 0);    // Green

        for (int i = 0; i < current_frame_->keypoints_.size(); i++){
            cv::circle(*img, current_frame_->keypoints_[i], 2, color, 2);
            int first_frame_match = current_frame_->matches_[i];
            cv::line(*img, current_frame_->keypoints_[i], first_frame_->keypoints_[first_frame_match], color);      
        }
        break;
    }
    
    case TRACKING:
    {
        for (int i = 0; i < current_frame_->keypoints_.size(); i++){
            cv::Scalar color(0, 255, 0);    // Green for the keypoints used for pose estimation

            // Change color for the just added keypoints, not used to estimate the pose
            if (i >= current_frame_->n_keypoints_used_for_pose_estimation_) color = cv::Scalar(0, 0, 255);  // Red for the new triangulated keypoints

            cv::circle(*img, current_frame_->keypoints_[i], 2, color, 2);
            int prev_frame_match = current_frame_->matches_[i];
            cv::line(*img, current_frame_->keypoints_[i], prev_frame_->keypoints_[prev_frame_match], color);    
        }

        for (int i = 0; i < current_frame_->candidate_keypoints_.size(); i++){
            cv::Scalar color(255, 0, 0);    // Blue for new candidate keypoints

            // Change color for the just added keypoints, not used to estimate the pose
            cv::circle(*img, current_frame_->candidate_keypoints_[i], 2, color, 2);
        }

        break;
    }
    default:
        break;
    }
}
