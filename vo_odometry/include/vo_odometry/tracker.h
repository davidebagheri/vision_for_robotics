#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/opencv.hpp>

#include "vo_odometry/frame.h"

class Tracker{
public:
    Tracker(const cv::FileStorage& params){
        patch_radius_ = (int) params["tracker.patch_radius"];
        n_keypoints_ = (int) params["tracker.n_keypoints"]; 
        nms_ = (int) params["tracker.nms"];
        ksize_ = (int) params["tracker.ksize"];
        k_ = (float) params["tracker.k"];
        tracking_err_th_ = (float) params["tracker.tracking_err_th"];
        max_dist_new_kp_ = (int) params["tracker.max_dist_new_kp"];
    }

    std::vector<cv::Point2f> extractKeypoints(const cv::Mat& image) const;

    void extractKeypoints(Frame* frame) const;

    void addNewKeypoints(Frame* frame) const;
    
    std::vector<cv::Point2f> selectKeypoints(const cv::Mat& harris, int nms_range, int n_keypoints) const; 

    void trackPoints(const cv::Mat& old_image,
                     const cv::Mat& new_image, 
                     const std::vector<cv::Point2f>& old_kpts, 
                     std::vector<cv::Point2f>& next_kpts,
                     std::vector<int>& matches);
    
    void trackPoints(Frame& old_frame, Frame& new_frame);
    
private:
    // Feature extraction params
    int patch_radius_;  
    int n_keypoints_;           // Number of keypoints to extract for each image
    int nms_;                   // Non-maximum-suppression range
    int ksize_;                 // Aperture parameter of the Sobel derivative used
    float k_;                   // Harris detector free parameter in the equation
    float tracking_err_th_;     // Lukas Kanade Tracker error threshold
    int max_dist_new_kp_;       // Max distance from a new extracted keypoint and the preexisting ones
    int n_max_kpts_;            // Maximum number of keypoints per frame
};


#endif
