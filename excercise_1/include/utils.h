#include <opencv2/opencv.hpp>
#include <math.h>

cv::Mat from_vec_to_cv_mat(const std::vector<std::vector<float>>& K);
cv::Mat to_cv_vec(std::vector<float> v);
cv::Mat to_homog(cv::Mat v);

cv::Mat crossProductMatrix(const std::vector<float>& vec);

cv::Mat poseVectorToTransformationMatrix(const std::vector<float>& pose);

cv::Vec2f projectPoint(const std::vector<float>& point,
                       const cv::Mat& K, 
                       const std::vector<float>& pose_CW);