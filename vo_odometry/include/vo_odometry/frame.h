#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>


class Frame {
public:
    Frame(const cv::Mat& image){
        if (image.channels() != 1){
            std::cerr << "Image must be grayscale!" << std::endl;
            return;
        }
        img_ = image;
    }


    void setPose(const cv::Matx33f& R, const cv::Vec3f& t){
        R_ = R;
        t_ = t;
    }

    // Image
    cv::Mat img_;

    // Features and projections
    std::vector<int> matches_;
    std::vector<cv::Point2f> keypoints_;
    std::vector<cv::Point3f> points_3d_;
    
    // Pose
    cv::Matx33f R_;
    cv::Vec3f t_;

    int n_pnp_kp_ = 0;
};





#endif
