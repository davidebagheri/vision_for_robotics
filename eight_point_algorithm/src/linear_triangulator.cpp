#include "eight_point_algorithm/linear_triangulator.h"


cv::Point3d LinearTriangulator::triangulatePoints(
        const cv::Point2f& point_1,
        const cv::Point2f& point_2,
        const cv::Matx34f& proj_mat_1,
        const cv::Matx34f& proj_mat_2){
        // Build the triangulation system matrix 
        cv::Matx<float, 6, 4> m;
        getTriangSystemMatrix(point_1, point_2, proj_mat_1, proj_mat_2, m);

        // Solve
        cv::SVD svd = cv::SVD(m, cv::SVD::FULL_UV);
        
        float den = svd.vt.at<float>(3, 3);
        cv::Point3d sol = {svd.vt.at<float>(3, 0) / svd.vt.at<float>(3, 3), 
                           svd.vt.at<float>(3, 1) / svd.vt.at<float>(3, 3), 
                           svd.vt.at<float>(3, 2) / svd.vt.at<float>(3, 3)};
        return sol;
}

void LinearTriangulator::getTriangSystemMatrix(
        const cv::Point2f& point_1,
        const cv::Point2f& point_2,
        const cv::Matx34f& proj_mat_1,
        const cv::Matx34f& proj_mat_2,
        cv::Matx<float,6,4>& sysmat){
        // Compute skew matrices for each homogenous pixel
        cv::Vec3f pix1_hom = toHomog(point_1);
        cv::Matx33f pix_1_mat = skew(pix1_hom);
        cv::Vec3f pix2_hom = toHomog(point_2);
        cv::Matx33f pix_2_mat = skew(pix2_hom);
        
        // Compute the two parts of the system matrix
        cv::Matx34f first_block = pix_1_mat * proj_mat_1;
        cv::Matx34f second_block = pix_2_mat * proj_mat_2;

        // Copy 
        for (int i = 0; i < 3; i++){
                for (int j = 0; j < 4; j++){
                        sysmat(i,j) = first_block(i,j);
                }
        }
        for (int i = 0; i < 3; i++){
                for (int j = 0; j < 4; j++){
                        sysmat(i+3,j) = second_block(i,j);
                }
        }
}