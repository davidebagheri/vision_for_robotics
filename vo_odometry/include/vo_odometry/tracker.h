#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "vo_odometry/frame.h"
#include "vo_odometry/camera.h"
#include "vo_odometry/utils.h"
#include "vo_odometry/map.h"

class Tracker{
public:
    Tracker(const cv::FileStorage& params){
        min_frame_keypoints_ = (float) params["tracker.min_frame_keypoints"];
        min_depth_ = (float) params["tracker.min_depth"];
        max_depth_ = (float) params["tracker.max_depth"];
    }

    std::vector<cv::Point2f> extractKeypoints(const Frame& frame, int max_n_keypoints, float quality_level, bool use_mask) const;

    void trackKeypoints(const Frame& previous_frame, Frame* next_frame) const;

    bool estimatePose(const Frame& previous_frame, Frame* cur_frame, const Map& map, const Camera& cam) const;

    void triangulateUnmatchedKeypoints(Frame* previous_frame, Frame* cur_frame, Map* map, const Camera& camera) const;

    void printPointCoordHisto(const std::vector<cv::Point3f>& points3d, const Frame& previous_frame) const;

    void addNewKeypoints(Frame* frame, const std::vector<cv::Point2f>& new_kpts) const;

private:
    int min_frame_keypoints_ = 50;  // Minimum number of keypoints in the current frame
    float min_depth_ = 3;            // Minimum z distance [m] for a triangulated point to be valid  
    float max_depth_ = 80;           // Maximum z distance [m] for a triangulated point to be valid
};


#endif
