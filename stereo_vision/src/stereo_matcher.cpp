#include "stereo_vision/stereo_matcher.h"

float StereoMatcher::SSD(const cv::Mat& patch_1, const cv::Mat& patch_2){
        cv::Mat diff = patch_1 - patch_2;
        cv::Mat square = diff.mul(diff);
        return cv::sum(square)[0];
}

bool StereoMatcher::findMatch(const cv::Mat& left, 
                            const cv::Point& patch_pos, 
                            const cv::Mat& right,
                            cv::Point* match_pos){
    // Compute correspondance check margins
    int col_search_start = std::max(left_margin_, patch_pos.x - d_max_);
    int col_search_end = std::max(left_margin_, patch_pos.x - d_min_);

    // Left patch to match
    cv::Mat left_patch = left.rowRange(patch_pos.y - upper_margin_, patch_pos.y + lower_margin_ + 1)
                        .colRange(patch_pos.x - left_margin_, patch_pos.x + right_margin_ + 1);

    float min_ssd = INFINITY;
    match_pos->x = -1;
    match_pos->y = patch_pos.y;
    std::vector<float> ssd_scores(col_search_end - col_search_start + 1, 0);

    for (int i = col_search_start; i < col_search_end; i++){
        // Get right corresponding patch 
        cv::Mat guess_corresp = right.rowRange(patch_pos.y - upper_margin_, patch_pos.y + lower_margin_ + 1)
                        .colRange(i - left_margin_, i + right_margin_ + 1);
        // Compute SSD score
        float cur_ssd = SSD(left_patch, guess_corresp);

        ssd_scores.emplace_back(cur_ssd);
        if (cur_ssd < min_ssd){
            min_ssd = cur_ssd;
            match_pos->x = i;
        }
    }

    if (match_pos->x > 0 and isGoodMatch(ssd_scores, min_ssd)) return true;
    else return false;
}

bool StereoMatcher::isGoodMatch(const std::vector<float>& ssd_scores, float min_ssd){
     // If the best score is at the edge of the matching range it means there is a local minimum outside
    // of the provided disparity
    if (ssd_scores[0] == min_ssd or ssd_scores.back() == min_ssd) return false;
    
    // Count the candidates similar to the minimum 
    int similar_to_min_count = 0;
    float ssd_threshold = is_good_match_ratio_ * min_ssd;

    for (auto& ssd_score : ssd_scores){
        if (ssd_score <= ssd_threshold and ssd_score != 0) similar_to_min_count++;
    }

    // Discard the disparity if there are too many candidates
    return similar_to_min_count < 3;
}

cv::Mat StereoMatcher::computeDisparity(const cv::Mat& left, const cv::Mat& right){
    cv::Mat disparity = cv::Mat::zeros(left.size(), CV_32FC1);

    #pragma omp parallel for 
        for (int i = upper_margin_; i < left.rows - lower_margin_; i++){
            for (int j = left_margin_; j < left.cols - right_margin_; j++){
                cv::Point patch_pos(j, i);
                cv::Point match_pos;
                if (findMatch(left, patch_pos, right, &match_pos)){
                    float disp = patch_pos.x - match_pos.x; // Compute disparity
                    
                }
            }
        }
    return disparity;
}

cv::Point3f StereoMatcher::computePointPositionFromMatch(const cv::Point2f& p0, const cv::Point2f& p1){
    cv::Vec3f p0_h = {p0.x, p0.y, 1};
    cv::Vec3f p1_h = {p1.x, p1.y, 1};

    // Compute matrix of the linear system A[scale0, scale1] = baseline
    cv::Vec3f A_col_0 = K0_inv_ * p0_h;
    cv::Vec3f A_col_1 = K1_inv_ * p1_h;
    cv::Matx32f A(A_col_0(0), A_col_1(0),
                  A_col_0(1), A_col_1(1),
                  A_col_0(2), A_col_1(2));
    cv::Matx31f b(baseline_, 0, 0);

    // Least Square solve
    cv::Matx21f scales;
    cv::solve(A, b, scales, cv::DECOMP_SVD);

    // Compute 3D point position
    cv::Point3f point_pos = scales(0,0) * K0_inv_ * p0_h;

    return point_pos;
}

std::vector<cv::Point3f> StereoMatcher::computePointcloud(const cv::Mat& left, const cv::Mat& right){
    std::vector<cv::Point3f> pointcloud;
    pointcloud.reserve(left.cols * left.rows);
    omp_lock_t pointcloudMutex;
    omp_init_lock(&pointcloudMutex);

    #pragma omp parallel for 
        for (int i = upper_margin_; i < left.rows - lower_margin_; i++){
            for (int j = left_margin_; j < left.cols - right_margin_; j++){
                cv::Point left_patch_pos(j, i);
                cv::Point right_patch_pos;
                if (findMatch(left, left_patch_pos, right, &right_patch_pos)){
                    cv::Point3f point = computePointPositionFromMatch(left_patch_pos, right_patch_pos);
                    omp_set_lock(&pointcloudMutex);
                    pointcloud.emplace_back(point);
                    omp_unset_lock(&pointcloudMutex);
                }
            }
        }
    pointcloud.shrink_to_fit();
    return pointcloud;
}