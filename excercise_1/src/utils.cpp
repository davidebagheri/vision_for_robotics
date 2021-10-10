#include "utils.h"

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

cv::Mat crossProductMatrix(const std::vector<float>& vec)
{
    assert(vec.size() == 3);

    cv::Mat res = cv::Mat::zeros(cv::Size(3, 3), CV_32FC1);

    res.at<float>(0,1) = -vec[2];
    res.at<float>(0,2) = vec[1];
    res.at<float>(1,0) = vec[2];
    res.at<float>(1,2) = -vec[0];
    res.at<float>(2,0) = -vec[1];
    res.at<float>(2,1) = vec[0];

    return res;
}

cv::Mat poseVectorToTransformationMatrix(const std::vector<float>& pose)
{
    assert(pose.size() == 6);

    // Get Rotation Matrix
    cv::Mat rot_mat;
    std::vector<float> rodrigues(pose.begin(), pose.begin()+3);
    cv::Mat rodrigues_cv(rodrigues);
    cv::Rodrigues(rodrigues_cv, rot_mat);

    // Fill result
    cv::Mat res(3, 4, CV_32FC1);

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            res.at<float>(i, j) = rot_mat.at<float>(i, j);
        }
    }
    res.at<float>(0,3) = pose[3];
    res.at<float>(1,3) = pose[4];
    res.at<float>(2,3) = pose[5];

    return res;
}

cv::Vec2f projectPoint(const std::vector<float>& point_W, const cv::Mat& K, const std::vector<float>& pose_CW)
{
    cv::Vec2f res;
    // Point in homogeneous form
    cv::Mat point_cv_W = to_cv_vec(point_W);
    cv::Mat h_point_cv_W = to_homog(point_cv_W);

    // World to Camera tranformation in matrix form
    cv::Mat T_CW = poseVectorToTransformationMatrix(pose_CW);

    // Project
    cv::Mat proj_point = K * T_CW * h_point_cv_W;

    // Scale normalization 
    res[0] = proj_point.at<float>(0) / proj_point.at<float>(2);
    res[1] = proj_point.at<float>(1) / proj_point.at<float>(2);
    
    return res;
}
