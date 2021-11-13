#include "pinhole_camera/camera_operations.h"

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

cv::Point2f projectPoint(const std::vector<float>& point_W, const cv::Mat& K, const std::vector<float>& pose_CW)
{
    cv::Point2f res;

    // Point in homogeneous form
    cv::Mat point_cv_W = to_cv_vec(point_W);
    cv::Mat h_point_cv_W = to_homog(point_cv_W);

    // World to Camera tranformation in matrix form
    cv::Mat T_CW = poseVectorToTransformationMatrix(pose_CW);

    // Project
    cv::Mat proj_point = K * T_CW * h_point_cv_W;

    // Scale normalization 
    res.x = proj_point.at<float>(0) / proj_point.at<float>(2);
    res.y = proj_point.at<float>(1) / proj_point.at<float>(2);

    return res;
}

cv::Point2f projectPoint(const cv::Point3f& point_W,  
                        const cv::Matx33f& K, 
                        const cv::Matx34f& T_CW)
{
    cv::Point2f res;

    // Point in homogeneous form
    cv::Vec4f homog_point_W = toHomog(point_W);

    // Project
    cv::Vec3f proj_point = K * T_CW * homog_point_W;

    // Scale normalization 
    res.x = proj_point[0] / proj_point[2];
    res.y = proj_point[1] / proj_point[2];

    return res;
}


cv::Point2f toDistortPixelCoords(const cv::Point2f& undist_point, 
                                const std::vector<float>& dist_params, 
                                const cv::Point2f& img_center)
{
    assert(dist_params.size() == 2);
    cv::Point2f diff = undist_point - img_center;
    float r2 = std::pow(diff.x, 2) + std::pow(diff.y, 2);

    cv::Point2f dist_point = (1 + dist_params[0] * r2 + dist_params[1] * r2 * r2) * diff + img_center;

    return dist_point;
}

cv::Mat undistortImage(cv::Mat dist_image, const std::vector<float>& dist_params)
{
    assert(dist_image.channels() == 1);  // Limit this function only to gray image, I am sorry :(
    int h = dist_image.rows;
    int w = dist_image.cols;
    cv::Point2f img_center = {h/2, w/2};

    cv::Mat und_image = cv::Mat::zeros(h, w, CV_8U);

    for (int i = 0; i < h; i++){
        for (int j = 0; j < w; j++){
            cv::Point2f undist_pixel = {i, j};

            // Nearest neighbor pixel
            cv::Point2i dist_pixel = toDistortPixelCoords(undist_pixel, dist_params, img_center);

            // Inbound check
            if (dist_pixel.x < 0 or dist_pixel.x > h) continue;
            if (dist_pixel.y < 0 or dist_pixel.y > w) continue;

            und_image.at<uint8_t>(i,j) = dist_image.at<uint8_t>(dist_pixel.x, dist_pixel.y);
        }
    }

    return und_image;
}