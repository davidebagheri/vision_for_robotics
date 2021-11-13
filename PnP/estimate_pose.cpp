#include "pinhole_camera/camera_operations.h"
#include "PnP/PnP.h"
#include "utils/data.h"
#include "utils/conversion.h"
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

    std::vector<std::vector<cv::Point2f>> points_2d_detections;
    for (const auto& point_2d_set : points_2d_std){
        std::vector<cv::Point2f> points_2d;
        for (int i = 0; i < points_2d_std[0].size(); i+=2){
            cv::Point2f point_2d = cv::Point2f(point_2d_set[i], point_2d_set[i+1]);
            points_2d.emplace_back(point_2d);
        }
        points_2d_detections.emplace_back(points_2d);
    }

    cv::Mat K_cv = from_vec_to_cv_mat(camera_matrix_std);
    cv::Matx33f camera_matrix = cv::Matx33f((float*)K_cv.data);
    cv::Matx33f camera_matrix_inv = camera_matrix.inv();

    // 3D visualizer
    Visualizer window("pose_estimation", true, 0.3);

    // Show 3D corners
    window.showPoints("keypoints", points_3d, 10);

    // DLT
    for (int i = 0; i < images_path.size(); i++){
        
        // Get frame
        std::string& img_path = images_path[i];
        cv::Mat frame = cv::imread(img_path);

        // Get points
        const std::vector<cv::Point2f>& points_2d = points_2d_detections[i];

        // Estimate pose
        cv::Matx34f T_CW = estimatePoseDLT(points_2d, points_3d, camera_matrix_inv);

        // Copy results in affine format 
        cv::Matx33f R;
        cv::Vec3f t;
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                R(i,j) = T_CW(i,j);
            }
        }
        for (int i = 0; i < 3; i++){
            t(i) = T_CW(i, 3);
        }

        // Compute reverse transformation (from camera to world)
        cv::Affine3f aff_T_WC(R.t(), -R.t() * t);

        // Project back points
        for (const auto& point_3d : points_3d){
            cv::Point2f projection = projectPoint(point_3d, camera_matrix, T_CW);
            cv::circle(frame, projection, 3, cv::Scalar(0,0,255), 3);
        }

        // Show results
        if (i == 0)
            window.showFrame("camera_frame", aff_T_WC, 0.1);
        else
            window.setWidgetPose("camera_frame", aff_T_WC);
        
        window.spinOnce(1, true);
        cv::imshow("Corners Projection", frame);
        cv::waitKey(60);
    }

    cv::destroyAllWindows();
}