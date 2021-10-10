#include <opencv2/opencv.hpp>
#include <math.h>

cv::Mat from_vec_to_cv_mat(const std::vector<std::vector<float>>& K);
cv::Vec3f to_cv_vec(std::vector<float> v);
cv::Vec4f to_homog(cv::Vec3f);

cv::Mat3f crossProductMatrix(const std::vector<float>& vec);

cv::Mat3f fromRodriguesToRotMatrix(const std::vector<float>& rodrigues);

cv::Mat poseVectorToTransformationMatrix(const std::vector<float>& pose);

cv::Vec2f projectPoint(const std::vector<float>& point,
                       const cv::Mat3f& K, 
                       const std::vector<float>& pose_CW);