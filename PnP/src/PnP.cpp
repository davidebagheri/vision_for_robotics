#include "PnP/PnP.h"


cv::Matx34f estimatePoseDLT(const std::vector<cv::Point2f>& point_2d, 
                            const std::vector<cv::Point3f>& point_3d,
                            const cv::Matx33f& camera_matrix_inv){
    
    assert(point_2d.size() == point_3d.size());

    // Get 2D-3D correspondence matrix
    cv::Mat Q = getLinearSysMat(point_2d, point_3d, camera_matrix_inv);

    // Solve SVD
    cv::SVD svd = cv::SVD(Q);

    // Get eigenvector corresponding to the last column of V and reshape
    cv::Mat M_vec = svd.vt.row(point_2d.size() - 1);
    cv::Mat M_vec_copy = M_vec.clone();
    cv::Mat M = cv::Mat(cv::Size(4, 3), CV_32FC1, M_vec_copy.data);

    // Enforce R in M = [R|t] to have determinant = 1
    if (M.at<float>(2,3) < 0) M = M * -1;

    // Solve Orthogonal Procrustes Problem to make R be SO(3) 
    cv::Mat R = M(cv::Range(0,3), cv::Range(0,3));
    cv::SVD svd_R = cv::SVD(R, cv::SVD::FULL_UV);
    cv::Mat R_tilde = svd_R.u * svd_R.vt;

    // Recover scale
    float scale = cv::norm(R_tilde) / cv::norm(R);

    cv::Mat t = M(cv::Range(0,3), cv::Range(3,4)) * scale;
    
    // Copy to final result
    cv::Matx34f pose;

    // Rotation
    pose(0,0) = R_tilde.at<float>(0, 0);
    pose(0, 1) = R_tilde.at<float>(0, 1);
    pose(0, 2) = R_tilde.at<float>(0, 2);
    pose(1, 0) = R_tilde.at<float>(1, 0);
    pose(1, 1) = R_tilde.at<float>(1, 1);
    pose(1, 2) = R_tilde.at<float>(1, 2);
    pose(2, 0) = R_tilde.at<float>(2, 0);
    pose(2, 1) = R_tilde.at<float>(2, 1);
    pose(2, 2) = R_tilde.at<float>(2, 2);
    // Translation
    pose(0, 3) = t.at<float>(0, 0);
    pose(1, 3) = t.at<float>(0, 1);
    pose(2, 3) = t.at<float>(0, 2);

    return pose;
}

cv::Mat getLinearSysMat(const std::vector<cv::Point2f>& points_2d,
                        const std::vector<cv::Point3f>& points_3d,
                        const cv::Matx33f& camera_matrix_inv)
{
    cv::Mat Q = cv::Mat::zeros(points_2d.size() * 2, 12, CV_32F);

    for (int i = 0; i < points_2d.size(); i++){
        int mat_row = i*2;
        const cv::Point2f& point_2d = points_2d[i];
        const cv::Point3f& point_3d = points_3d[i];
        const cv::Vec4f h_point3d = {point_3d.x, point_3d.y, point_3d.z, 1};
        cv::Vec3f calib_coords = fromPixelToCalibCoords(point_2d, camera_matrix_inv);
        
        Q.at<cv::Vec4f>(mat_row, 0) = h_point3d;
        Q.at<cv::Vec4f>(mat_row, 2) = h_point3d * (-calib_coords[0]);
        Q.at<cv::Vec4f>(mat_row+1, 1) = h_point3d;
        Q.at<cv::Vec4f>(mat_row+1, 2) = h_point3d * (-calib_coords[1]);
    }

    return Q;
}


cv::Vec3f fromPixelToCalibCoords(const cv::Point2f& pixel_coords, 
                                const cv::Matx33f& camera_matrix_inv)
{
    cv::Vec3f homog_px_coords = {pixel_coords.x, pixel_coords.y, 1};
    cv::Vec3f homog_calib_coords = camera_matrix_inv * homog_px_coords;
    return homog_calib_coords;
}