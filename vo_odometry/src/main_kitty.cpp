#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils/data.h>
#include <utils/visualizer.h>

#include "vo_odometry/vo_system.h"


int main(int argc, char** argv){
    /*if (argc != 2){
        std::cerr << "Usage: ./main_kitty <path-to-kitty-dataset>" << std::endl;
        return -1;
    }*/

    // Read images
    //std::string path_to_img_folder = std::string(argv[1]);
    std::string path_to_img_folder = "/home/dbagheri/Desktop/vision_for_robotics/vo_odometry/data/kitti/05/image_0";
    std::vector<std::string> imgpaths = get_images_path(path_to_img_folder, "/*.png");
    std::cout << "Found " << imgpaths.size() << " images to process." << std::endl;

    // Initialize system
    cv::FileStorage params("/home/dbagheri/Desktop/vision_for_robotics/vo_odometry/cfg/kitty.yaml", cv::FileStorage::READ);
    VOSystem vo_system(params);

    Visualizer vis("Visualizer");
    vis.spinOnce();
    // Loop
    for (int i = atoi(argv[1]); i < imgpaths.size(); i++){
        cv::Mat image = cv::imread(imgpaths[i]);
        cv::Mat gray;
        cv::cvtColor(image, gray, CV_BGR2GRAY);

        // Visual odometry
        vo_system.processImage(gray);


        // Visualize Pose
        cv::Matx33f R;
        cv::Vec3f t;

        if (vo_system.getPose(R, t)){
            vis.showFrame("camera", R, t, 0.6);
        }

        // Visualize 3d points
        std::vector<cv::Point3f> points_3d;
        if (vo_system.get3dPoints(points_3d)){
            vis.showPoints("pts", points_3d);
            std::cout << "points shown" << std::endl;
        }

        // Visualize features
        vo_system.visualize(image);
        cv::imshow("visual odometry", image);
        
        
        vis.spinOnce();

        if (cv::waitKey(30) == 27) break;

                // Remove later
        //if (vo_system.getState() == TRACKING){
        //    break;
        //}


    }
    vis.spin();
    cv::waitKey();
    cv::destroyAllWindows();

    return 0;
}
