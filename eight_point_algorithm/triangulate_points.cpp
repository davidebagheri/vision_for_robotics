#include <iostream>
#include <cstdlib>
#include "eight_point_algorithm/linear_triangulator.h"


int main(){
    // Data
    cv::Matx34f proj_mat1(500, 0, 320, 0, 0, 500, 240, 0, 0, 0, 1, 0);
    cv::Matx34f proj_mat2(500, 0, 320, -100, 0, 500, 240, 0, 0, 0, 1, 0);

    // Random 3d point
    
    cv::Vec4f P ={2.0, 
                  2.0, 
                  2.0,
                    1};

    // Compute projections
    cv::Vec3f proj_1 = proj_mat1 * P;
    cv::Vec3f proj_2 = proj_mat2 * P;
    cv::Point2f pix_proj_1(proj_1[0] / proj_1[2], proj_1[1] / proj_1[2]);
    cv::Point2f pix_proj_2(proj_2[0] / proj_1[2], proj_2[1] / proj_1[2]);

    LinearTriangulator lin_triang;

    lin_triang.triangulatePoints(pix_proj_1, pix_proj_2, proj_mat1, proj_mat2);
}