#include <iostream>
#include <opencv2/opencv.hpp>
#include "utils/conversion.h"

class EightPointAlgorithm{
public:
    cv::Matx33d computeFoundamentalMatrix(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matched_pts);

    cv::Matx33d computeFoundamentalMatrixNormalized(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matched_pts);

    cv::Mat getSystemMatrixForFM(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matched_pts);

    cv::Mat forceMatToZeroDet(const cv::Mat& matrix);

    double computeGeometricError(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matches, const cv::Matx33d& F);

    double computeAlgebraicError(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matches, const cv::Matx33d& F);

};