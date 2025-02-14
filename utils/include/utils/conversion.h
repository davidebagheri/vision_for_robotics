#include <opencv2/opencv.hpp>

cv::Mat from_vec_to_cv_mat(const std::vector<std::vector<float>>& K);

cv::Mat to_cv_vec(std::vector<float> v);

cv::Mat to_homog(cv::Mat v);

cv::Vec4f toHomog(const cv::Point3f& point_3d);

cv::Vec3f toHomog(const cv::Point2f& point_2d);

cv::Matx33f skew(const cv::Vec3f& vec);

