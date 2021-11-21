#include <vector>
#include <opencv2/opencv.hpp>
#include "utils/conversion.h"


class LinearTriangulator{
public:
    cv::Point3d triangulatePoints(
        const std::pair<cv::Point2f, cv::Point2f>& correspondences,
        const cv::Matx34f& proj_mat_1,
        const cv::Matx34f& proj_mat_2);
    
    void getTriangSystemMatrix(
        const std::pair<cv::Point2f, cv::Point2f>& correspondences,
        const cv::Matx34f& proj_mat_1,
        const cv::Matx34f& proj_mat_2,
        cv::Matx<float,6,4>& sysmat);
};