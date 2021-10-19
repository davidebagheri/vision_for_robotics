#include "PnP/PnP.h"
#include "utils/data.h"
#include "utils/visualizer.h"

int main(int argc, char** argv){
    if (argc != 2)
    {
        std::cout << "Data path must be passed as argument" << std::endl;
        return -1;
    }

    // Data paths
    std::string data_path(argv[1]);
    std::string points_2d_path = data_path + "/detected_corners.txt";
    std::string points_3d_path = data_path + "/p_W_corners.txt";
    std::string camera_matrix_path = data_path + "/K.txt";
    std::string images_folder_path = data_path + "/images_undistorted";

    // Get data 
    std::vector<std::string> images_path = get_images_path(images_folder_path);
    std::vector<std::vector<float>> points_2d_std = read_csv<float>(points_2d_path, ' ');
    std::vector<std::vector<float>> points_3d_std = read_csv<float>(points_3d_path, ',');
    std::vector<std::vector<float>> camera_matrix_std = read_csv<float>(camera_matrix_path, ',');
    
    // Take data to the proper form
    std::vector<cv::Point3f> points_3d;
    for (const auto& point_3d_std : points_3d_std){
        cv::Point3f point_3d(point_3d_std[0], point_3d_std[1], point_3d_std[2]);
        points_3d.emplace_back(point_3d);
    }
    std::vector<cv::Point2f> points_2d;
    for (const auto& point_2d_set : points_2d_std){
        for (int i = 0; i < points_2d_std.size(); i+=2){
            cv::Point2f point_2d = cv::Point2f(point_2d_set[i], point_2d_set[i+1]);
            points_2d.emplace_back(point_2d);
        }
    }
    //cv::Mat K_cv = from_vec_to_cv_mat(camera_matrix_std);
    //cv::Matx33f camera_matrix = cv::Matx33f(K_cv.data);
    cv::Matx33f camera_matrix_inv;// = camera_matrix.inv();

    // DLT
    for (int i = 0; i < images_path.size(); i++){
        std::string& img_path = images_path[i];
        cv::Mat frame = cv::imread(img_path);

        estimatePoseDLT(points_2d, points_3d, camera_matrix_inv);

        cv::imshow("prova", frame);
        cv::waitKey(30);
    }

    cv::destroyAllWindows();
}