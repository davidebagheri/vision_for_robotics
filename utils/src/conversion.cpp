#include "utils/conversion.h"

cv::Mat from_vec_to_cv_mat(const std::vector<std::vector<float>>& K)
{
    int rows = K.size();
    int cols = K[0].size();
    cv::Mat K_cv(rows, cols, CV_32FC1);

    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            K_cv.at<float>(i, j) = K[i][j];
        }
    }
    return K_cv;
}

cv::Mat to_cv_vec(std::vector<float> v){
    assert(v.size() == 3);
    cv::Mat cv_vec(3, 1, CV_32FC1);

    for (int i = 0; i < 3; i++){
        cv_vec.at<float>(i) = v[i];
    }
    
    return cv_vec;
}

cv::Mat to_homog(cv::Mat v){
    cv::Mat res(4, 1, CV_32FC1);
    for (int i = 0; i < 3; i++){
        res.at<float>(i) = v.at<float>(i);
    }
    res.at<float>(3) = 1.0;
    return res;
}

cv::Vec4f toHomog(const cv::Point3f& point_3d){
    return cv::Vec4f(point_3d.x, point_3d.y, point_3d.z, 1);
}