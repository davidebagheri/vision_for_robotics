#include "simple_keypoint_tracker/tracker.h"


std::vector<cv::Point> Tracker::extractKeypoints(const cv::Mat& image, int n_keypoints){
    // Compute harris score
    cv::Mat harris;
    cv::cornerHarris(image, harris, patch_size_, 3, k_harris_);

    // Select Keypoints
    std::vector<cv::Point> keypoints = selectKeypoints(harris, 10, n_keypoints);

    return keypoints;
}

std::vector<cv::Point> Tracker::selectKeypoints(cv::Mat& harris, int nms_range, int n_keypoints){
    std::vector<cv::Point> keypoints;


    for (int i = 0; i < n_keypoints; i++){
        // Find current max score
        cv::Point min_loc, max_loc;
        double min, max;
        cv::minMaxLoc(harris, &min, &max, &min_loc, &max_loc);

        // Select keypoints in a descrtiptable position
        if (max_loc.x > left_margin_ and max_loc.x < harris.cols - right_margin_
        and max_loc.y > upper_margin_ and max_loc.y < harris.rows - lower_margin_){
            keypoints.emplace_back(max_loc);
        }

        // Set neighbouring pixels to zero
        int min_x = std::max(0, max_loc.x - nms_range);
        int max_x = std::min(harris.cols, max_loc.x + nms_range);
        int min_y = std::max(0, max_loc.y - nms_range);
        int max_y = std::min(harris.rows, max_loc.y + nms_range);
        harris(cv::Range(min_y, max_y), cv::Range(min_x, max_x)) = 0;
    }

    return keypoints;
}

float Tracker::keypointDistanceSSD(const cv::Mat& kp1, const cv::Mat& kp2){
        cv::Mat diff = kp1 - kp2;
        cv::Mat square = diff.mul(diff);
        return cv::sum(square)[0];
}

std::vector<int> Tracker::matchKeypoints(const std::vector<cv::Point>& kps1, 
                                                        const cv::Mat& image1, 
                                                        const std::vector<cv::Point>& kps2, 
                                                        const cv::Mat& image2){
    
    cv::Mat matches_matrix = cv::Mat::zeros(cv::Size(kps2.size(), kps1.size()), CV_32FC1);

    // Compute distance among descriptors in a brute force approach
    for (int idx_1 = 0; idx_1 < kps1.size(); idx_1++){
        for (int idx_2 = 0; idx_2 < kps2.size(); idx_2++){
            cv::Point kp1 = kps1[idx_1];
            cv::Point kp2 = kps2[idx_2];

            cv::Mat descr_1 = image1(cv::Range(kp1.y-upper_margin_, kp1.y+lower_margin_),
                                    cv::Range(kp1.x-left_margin_, kp1.x+right_margin_));
            cv::Mat descr_2 = image2(cv::Range(kp2.y-upper_margin_, kp2.y+lower_margin_),
                                    cv::Range(kp2.x-left_margin_, kp2.x+right_margin_));
            matches_matrix.at<float>(idx_1, idx_2) = keypointDistanceSSD(descr_1, descr_2);
        }
    }

    // Get the best matches
    std::vector<std::pair<int, double>> best_matches;
    best_matches.reserve(kps1.size());
    double min_dist = INFINITY;
    
    for (int idx_1 = 0; idx_1 < kps1.size(); idx_1++){
        cv::Point min_loc, max_loc;
        double min, max;
        cv::minMaxLoc(matches_matrix.row(idx_1), &min, &max, &min_loc, &max_loc);
        best_matches.emplace_back(std::pair<int, double>(min_loc.x, min));

        if (min > 0) min_dist = std::min(min_dist, min);
    }

    // Apply threshold
    std::vector<int> matches12(kps1.size(), -1);
    std::vector<bool> matched2(kps2.size(), false);
    double threshold = lambda_ * min_dist;
    int count = 0;
    for (int i = 0; i < kps1.size(); i++){
        if (!matched2[best_matches[i].first] and best_matches[i].second < threshold){
            matches12[i] = best_matches[i].first;
            matched2[best_matches[i].first] = true;
            count++;
        } 
    }
    std::cout << "n matches: " << count << std::endl;
    return matches12;
}

/*std::vector<cv::Mat> Tracker::sobelXY(const cv::Mat& image){
    // x sobel kernel
    float x_sobel[] = {-1, 0, 1,
                       -2, 0, 2,
                       -1, 0, 1};
    cv::Mat x_kernel(3, 3, CV_32F, x_sobel);

    // y sobel kernel
    float y_sobel[] = {-1, -2, -1,
                        0,  0,  0,
                        1,  2,  1};
    cv::Mat y_kernel(3, 3, CV_32F, y_sobel);


    //convolove
    cv::Mat x_responses;
    cv::Mat y_responses;
    
    cv::filter2D(image, x_responses, image.depth(), x_kernel);
    cv::filter2D(image, y_responses, image.depth(), y_kernel);

    return std::vector<cv::Mat>({x_responses, y_responses});
}*/

/*cv::Mat Tracker::cornerHarris(const cv::Mat& image){
    std::cout << "initial image: " << image.size() << std::endl;
    // Sobel 
    std::vector<cv::Mat> sobel_xy = sobelXY(image);
    cv::Mat Ix = sobel_xy[0];
    cv::Mat Iy = sobel_xy[1];

    // Compute multiplications
    cv::Mat Ixx = Ix.mul(Ix);
    cv::Mat Iyy = Iy.mul(Iy);
    cv::Mat Ixy = Ix.mul(Iy);

    // Sum over the kernel size
    cv::Mat sumIxx, sumIyy, sumIxy;
    cv::Mat sum_kernel = cv::Mat::ones(cv::Size(patch_size_, patch_size_), CV_32FC1);
    cv::filter2D(Ixx, sumIxx, Ixx.depth(), sum_kernel);
    cv::filter2D(Iyy, sumIyy, Iyy.depth(), sum_kernel);
    cv::filter2D(Ixy, sumIxy, Ixy.depth(), sum_kernel);
    std::cout << "Ixx: " << Ixx.size() << std::endl;
    std::cout << "Iyy: " << Iyy.size() << std::endl;
    std::cout << "Ixy: " << Ixy.size() << std::endl;
    
    // Compute determinant and trace
    cv::Mat detM = sumIxx.mul(sumIyy) - sumIxy.mul(sumIxy);
    cv::Mat traceM = sumIxx + sumIyy;
    
    // Harris
    cv::Mat harris = detM - k_harris_ * traceM.mul(traceM);

    std::cout << harris.cols << "  " << harris.rows << std::endl;

    for (int i=0; i < harris.rows; i++){
        for (int j=0; j < harris.cols; j++){
            if (harris.at<float>(i,j) < 0) harris.at<float>(i,j) = 0.0;
        }
    }


    return harris;
}*/