#include "utils.h"

cv::Mat from_vec_to_cv_mat(const std::vector<std::vector<float>>& K)
{
    int rows = K.size();
    int cols = K[0].size();
    cv::Mat K_cv(rows, cols, CV_32F);

    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            K_cv.at<float>(i, j) = K[i][j];
        }
    }
    return K_cv;
}

cv::Vec3f to_cv_vec(std::vector<float> v){
    assert(v.size() == 3);
    cv::Vec3f cv_vec;
    for (int i = 0; i < 3; i++){
        cv_vec[i] = v[i];
    }
    return cv_vec;
}

cv::Vec4f to_homog(cv::Vec3f v){
    cv::Vec4f res;
    for (int i = 0; i < 3; i++){
        res[i] = v[i];
    }
    res[3] = 1.0;
    return res;
}

cv::Mat3f crossProductMatrix(const std::vector<float>& vec)
{
    assert(vec.size() == 3);

    cv::Mat res = cv::Mat::zeros(cv::Size(3, 3), CV_32F);

    res.at<float>(0,1) = -vec[2];
    res.at<float>(0,2) = vec[1];
    res.at<float>(1,0) = vec[2];
    res.at<float>(1,2) = -vec[0];
    res.at<float>(2,0) = -vec[1];
    res.at<float>(2,1) = vec[0];

    return res;
}


cv::Mat3f fromRodriguesToRotMatrix(const std::vector<float>& rodrigues)
{   
    assert(rodrigues.size() == 3);
    float angle = (float)std::sqrt(std::pow(rodrigues[0], 2) + 
                            std::pow(rodrigues[0], 2) +
                            std::pow(rodrigues[0], 2));
    cv::Mat3f cp_rodigues = crossProductMatrix(rodrigues);
    cv::Mat3f res = cv::Mat::eye(cv::Size(3, 3), CV_32F)
                    + std::sin(angle) * cp_rodigues
                    + (1 - std::cos(angle) * cp_rodigues * cp_rodigues);
    return res;
}

cv::Mat poseVectorToTransformationMatrix(const std::vector<float>& pose)
{
    assert(pose.size() == 6);

    // Get Rotation Matrix
    std::vector<float> rodrigues(pose.begin()+3, pose.end());
    cv::Mat3f rot_mat = fromRodriguesToRotMatrix(rodrigues);

    // Fill result
    cv::Mat res(3, 4, CV_32F);

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            res.at<float>(i, j) = rot_mat.at<float>(i, j);
        }
    }
    res.at<float>(3, 0) = pose[3];
    res.at<float>(4, 0) = pose[4];
    res.at<float>(5, 0) = pose[5];

    return res;
}

cv::Vec2f projectPoint(const std::vector<float>& point_W, const cv::Mat3f& K, const std::vector<float>& pose_CW)
{
    cv::Vec2f res;
    // Point in homogeneous form
    cv::Vec3f point_cv_W = to_cv_vec(point_W);
    cv::Vec4f h_point_cv_W = to_homog(point_cv_W);

    // World to Camera tranformation in matrix form
    cv::Mat T_CW = poseVectorToTransformationMatrix(pose_CW);

    // Project
    cv::Vec3f proj_point = K.dot(T_CW.dot(h_point_cv_W));

    // Scale normalization 
    res[0] = proj_point[0] / proj_point[2];
    res[1] = proj_point[1] / proj_point[2];

    return res;
}
