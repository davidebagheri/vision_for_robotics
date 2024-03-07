#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

class Frame {
public:
    Frame() = default;
    ~Frame() = default;
    
    Frame(const cv::Mat& image, int frame_n){
        img_ = image;
        frame_n_ = frame_n;
    }

    // Image
    int frame_n_;
    cv::Mat img_;

    // Features and projections
    std::vector<cv::Point2f> keypoints_;            // 2d visual features location
    std::vector<int> matches_;                      // Matches with the previous frame 
    std::vector<int> keypoints_map_id_;             // Id mapping keypoints and map 3d points 
    std::vector<cv::Point2f> candidate_keypoints_;  // Untracked 2d keypoints
    std::vector<int> candidate_keypoints_matches_;  // Untracked 2d keypoints matches with the previous frame

    int n_keypoints_used_for_pose_estimation_;
    
    // Pose
    cv::Affine3f pose;

};





#endif
