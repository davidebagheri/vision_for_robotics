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
                std::cout << "Initialization finished with n inliers: " << current_frame_->keypoints_.size() << std::endl;
                // Extract new keypoints 
                tracker_->addNewKeypoints(current_frame_);
                std::cout << "Added points, now n keypoints are " << current_frame_->keypoints_.size() << std::endl;

                state_ = TRACKING;
            } 

            prev_frame_ = current_frame_;
            break;
        }
        case TRACKING:
        {
            current_frame_ = new Frame(image);
            
            // Track previous keypoints
            tracker_->trackPoints(*prev_frame_, *current_frame_);

            std::cout << "Tracked " << prev_frame_->keypoints_.size() << " " << prev_frame_->matches_.size() << " " << prev_frame_->points_3d_.size() << " keypoints!\n";
            std::cout << "Tracked " << current_frame_->keypoints_.size() << " " << current_frame_->matches_.size() << " " << current_frame_->points_3d_.size() << " keypoints!\n";

            // Recover pose through Pnp
            std::vector<int> inlier_matches;
            tracker_->estimatePose(*prev_frame_, *current_frame_, inlier_matches, *camera_);

            // Triangulate new points
            std::vector<cv::Point3f> addedPt3d = tracker_->triangulateAddedKeypoints(*prev_frame_, *current_frame_, *camera_);

            // Filter out outliers and store the triangulated points
            std::vector<cv::Point2f> new_kpts;
            std::vector<cv::Point3f> new_pt3d;

            for (int i = 0; i < inlier_matches.size(); i++){
                new_kpts.emplace_back(current_frame_->keypoints_[i]);
                new_pt3d.emplace_back(prev_frame_->points_3d_[i]);
            }

            current_frame_->n_pnp_kp_ = inlier_matches.size();

            // Add newly triangulated Points
            int n_orig_pt3d = prev_frame_->points_3d_.size();

            for (int i = n_orig_pt3d; i < prev_frame_->keypoints_.size(); i++){
                if (prev_frame_->matches_[i] != -1){
                    new_kpts.emplace_back(prev_frame_->keypoints_[i]);
                    new_pt3d.emplace_back(addedPt3d[i-n_orig_pt3d]);
                }
            }

            current_frame_->keypoints_ = new_kpts;
            current_frame_->points_3d_ = new_pt3d;
            current_frame_->matches_ = std::vector<int>(current_frame_->keypoints_.size(), -1);

            // Extract new keypoints if necessary
            tracker_->addNewKeypoints(current_frame_);
            //std::cout << "Added points, now n keypoints are " << current_frame_->keypoints_.size() << std::endl;

            prev_frame_ = current_frame_;
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

const State& VOSystem::getState(){
    return state_;
}

bool VOSystem::getPose(cv::Matx33f& R, cv::Vec3f& t){
    if (current_frame_ != nullptr){
        R = current_frame_->R_;
        t = current_frame_->t_;
        return true;
    } 
    return false;
}

bool VOSystem::get3dPoints(std::vector<cv::Point3f>& points_3d){
    if (current_frame_ != nullptr and current_frame_->points_3d_.size() != 0){
        points_3d = current_frame_->points_3d_;
        return true;
    }
    return false;
}

void VOSystem::visualize(cv::Mat img){
    for (int i = 0; i < current_frame_->keypoints_.size(); i++){
            cv::Scalar color;
            if (i < current_frame_->n_pnp_kp_-1){
                color = cv::Scalar(0,255,0);
            } else {
                color = cv::Scalar(0,0,255);
            }
            cv::circle(img, current_frame_->keypoints_[i], 2, color, 2);            
    }
}
