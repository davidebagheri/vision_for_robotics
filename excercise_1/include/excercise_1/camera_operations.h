#include <opencv2/opencv.hpp>
#include <math.h>

#include "utils/conversion.h"

cv::Mat crossProductMatrix(const std::vector<float>& vec);

cv::Mat poseVectorToTransformationMatrix(const std::vector<float>& pose);

cv::Vec2f projectPoint(const std::vector<float>& point,
                       const cv::Mat& K, 
                       const std::vector<float>& pose_CW);

cv::Point2f toDistortPixelCoords(const cv::Point2f& undist_point, 
                                const std::vector<float>& dist_params, 
                                const cv::Point2f& img_center);

cv::Mat undistortImage(cv::Mat dist_image, const std::vector<float>& dist_params);