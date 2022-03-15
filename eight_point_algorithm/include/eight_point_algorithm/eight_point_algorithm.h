#include <iostream>
#include <opencv2/opencv.hpp>
#include "eight_point_algorithm/linear_triangulator.h"
#include "utils/conversion.h"

class EightPointAlgorithm{
public:
    cv::Matx33d computeFoundamentalMatrix(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2);

    cv::Matx33d computeFoundamentalMatrixNormalized(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2);

    cv::Mat getSystemMatrixForFM(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2);

    cv::Mat forceMatToZeroDet(const cv::Mat& matrix);

    double computeGeometricError(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2, const cv::Matx33d& F);

    double computeAlgebraicError(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2, const cv::Matx33d& F);

    cv::Matx33d computeNormMatrix(const std::vector<cv::Vec3d>& points);

    cv::Matx33d estimateEssentialMatrix(const std::vector<cv::Point2f>& points_1, const std::vector<cv::Point2f>& points_2, const cv::Matx33d& K1, const cv::Matx33d& K2);

    std::vector<cv::Matx34d> decomposeEssentialMatrix(const cv::Matx33d& E);

    cv::Matx34d disambiguateRelativePose(
        const std::vector<cv::Matx34d>& possible_T_C1_C2,
        const cv::Matx33d& K1,
        const cv::Matx33d& K2,
        const std::vector<cv::Point2f>& points_1,
        const std::vector<cv::Point2f>& points_2);

    cv::Point3d triangulatePoints(
        const cv::Point2f& point_1,
        const cv::Point2f& point_2,
        const cv::Matx34f& proj_mat_1,
        const cv::Matx34f& proj_mat_2){
            return linear_triangulator_.triangulatePoints(
                point_1, point_2, proj_mat_1, proj_mat_2);
        }
private:
    LinearTriangulator linear_triangulator_;
};