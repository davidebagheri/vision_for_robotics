#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils/data.h>
#include <utils/visualizer.h>

#include "vo_odometry/vo_system.h"

cv::Mat image;

void onMouse(int event, int x, int y, int flags, void* param)
{
    char text[100];
    cv::Mat img2, img3;

    img2 = image.clone();

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        cv::Vec3b p = img2.at<cv::Vec3b>(y,x);
        sprintf(text, "R=%d, G=%d, B=%d", p[2], p[1], p[0]);
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        cvtColor(image, img3, cv::COLOR_BGR2HSV);
        cv::Vec3b p = img3.at<cv::Vec3b>(y,x);
        sprintf(text, "H=%d, S=%d, V=%d", p[0], p[1], p[2]);
    }
    else
        sprintf(text, "x=%d, y=%d", x, y);

    putText(img2, text, cv::Point(5,15), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,0,0));
    imshow("visual_odometry", img2);
}

int main(int argc, char** argv){
    if (argc != 3){
        std::cerr << "Usage: ./main_kitty <path-to-kitty-dataset> <calibration>" << std::endl;
        return -1;
    }

    // Read images
    std::string path_to_img_folder = std::string(argv[1]);
    std::vector<std::string> imgpaths = get_images_path(path_to_img_folder, "/*.png");
    std::cout << "Found " << imgpaths.size() << " images to process." << std::endl;

    // Initialize system
    std::string path_to_calib = std::string(argv[2]);
    cv::FileStorage params(path_to_calib, cv::FileStorage::READ);
    VOSystem vo_system(params);

    // Loop
    for (int i = 0; i < imgpaths.size(); i++){
        // Load and convert
        image = cv::imread(imgpaths[i]);
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // Visual odometry
        vo_system.processImage(gray);

        // Visualize 
        vo_system.visualize(&image);

        cv::setMouseCallback("visual_odometry", onMouse, 0);
        cv::imshow("visual_odometry", image);

        if (cv::waitKey(0) == 27) break;
    }

    cv::destroyAllWindows();

    return 0;
}
