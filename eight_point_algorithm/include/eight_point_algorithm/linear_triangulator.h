#include <vector>
#include <opencv2/opencv.hpp>
#include "utils/conversion.h"


class LinearTriangulator{
public:
    cv::Point3d triangulatePoints(
        const cv::Point2f& point_1,
        const cv::Point2f& point_2,
        const cv::Matx34f& proj_mat_1,
        const cv::Matx34f& proj_mat_2);
    
    void getTriangSystemMatrix(
        const cv::Point2f& point_1,
        const cv::Point2f& point_2,
        const cv::Matx34f& proj_mat_1,
        const cv::Matx34f& proj_mat_2,
        cv::Matx<float,6,4>& sysmat);
};