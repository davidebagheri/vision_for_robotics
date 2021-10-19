#include <opencv2/opencv.hpp>


cv::Matx34f estimatePoseDLT(const std::vector<cv::Point2f>& point_2d, 
                            const std::vector<cv::Point3f>& point_3d,
                            const cv::Matx33f& camera_matrix_inv);

cv::Mat getLinearSysMat(const std::vector<cv::Point2f>& point_2d,
                        const std::vector<cv::Point3f>& point_3d,
                        const cv::Matx33f& camera_matrix_inv);

cv::Vec3f fromPixelToCalibCoords(const cv::Point2f& pixel_coords, 
                                const cv::Matx33f& camera_matrix_inv);