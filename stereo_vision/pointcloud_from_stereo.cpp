#include <iostream>
#include "stereo_vision/stereo_matcher.h"
#include "utils/data.h"
#include "utils/conversion.h"
#include "utils/visualizer.h"

int main(int argc, char** argv){
    if (argc != 2){
        std::cout << "Data path must be passed as argument" << std::endl;
    }

    std::string data_path(argv[1]);
    std::vector<std::string> left_images_paths = get_images_path(data_path + "/left", "/*.png");
    std::vector<std::string> right_images_paths = get_images_path(data_path + "/right", "/*.png");
    std::string camera_matrix_path = data_path + "/K.txt";

    // Get image
    cv::Mat left_img_rgb = cv::imread(left_images_paths[0]);
    cv::Mat right_img_rgb = cv::imread(right_images_paths[0]);
    cv::Mat left_img, right_img;
    cv::cvtColor(left_img_rgb, left_img, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_img_rgb, right_img, cv::COLOR_BGR2GRAY);
    left_img.convertTo(left_img, CV_32FC1);
    right_img.convertTo(right_img, CV_32FC1);

    // Get camera matrixes
    std::vector<std::vector<float>> camera_matrix_std = read_csv<float>(camera_matrix_path, ',');
    cv::Mat K_cv = from_vec_to_cv_mat(camera_matrix_std);
    cv::Matx33f camera_matrix = cv::Matx33f((float*)K_cv.data);
    cv::Matx33f camera_matrix_inv = camera_matrix.inv();

    // Baseline
    float baseline = 0.54;

    StereoMatcher stereo_matcher(camera_matrix, camera_matrix, baseline);
    
    std::vector<cv::Point3f> points_3d = stereo_matcher.computePointcloud(left_img, right_img);

    // 3D visualizer
    Visualizer window("Pointcloud visualization", true, 1);
    window.showPoints("keypoints", points_3d, 5);
    window.spin();
}