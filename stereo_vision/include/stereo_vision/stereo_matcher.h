#include <vector>
#include <opencv2/opencv.hpp>
#include <omp.h>

class StereoMatcher {
public:
    StereoMatcher(cv::Matx33f K0, cv::Matx33f K1, float baseline) : K0_(K0), K1_(K1), baseline_(baseline){
        // Patch params
        left_margin_ = (patch_size_-1) / 2;
        right_margin_ = patch_size_ - left_margin_ - 1;
        upper_margin_ = (patch_size_-1) / 2;
        lower_margin_ = patch_size_ - upper_margin_ - 1;

        // Camera params
        K0_inv_ = K0_.inv();
        K1_inv_ = K1_.inv();
    };

    cv::Mat computeDisparity(const cv::Mat& left, const cv::Mat& right);

    bool findMatch(const cv::Mat& left, const cv::Point& patch_pos, const cv::Mat& right, cv::Point* match_pos);

    bool isGoodMatch(const std::vector<float>& ssd_scores, float min_ssd);

    float SSD(const cv::Mat& patch_1, const cv::Mat& patch_2);

    cv::Point3f computePointPositionFromMatch(const cv::Point2f& p0, const cv::Point2f& p1);

    std::vector<cv::Point3f> computePointcloud(const cv::Mat& left, const cv::Mat& right);

private:
    // Params
    int d_min_ = 5; // minimum disparity
    int d_max_ = 50; // maximum disparity
    int patch_size_ = 16;
    int left_margin_, right_margin_, upper_margin_, lower_margin_;
    float is_good_match_ratio_ = 1.5;
    
    // Camera params
    cv::Matx33f K0_, K0_inv_;     // left camera
    cv::Matx33f K1_, K1_inv_;     // Right camera
    float baseline_;
};
