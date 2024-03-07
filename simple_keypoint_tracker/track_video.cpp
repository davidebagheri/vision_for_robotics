#include <iostream>
#include <thread>
#include <chrono>
#include "simple_keypoint_tracker/tracker.h"
#include "utils/data.h"

int main(int argc, char** argv){
    if (argc != 2){
        std::cout << "Data path must be passed as argument" << std::endl;
        return -1;
    }

    std::string data_path(argv[1]);
    std::vector<std::string> images_paths = get_images_path(data_path, "/*.png");

    Tracker tracker(17, 0.01, 10);

    cv::Mat gray_old = cv::imread(images_paths[0], 0);
    gray_old.convertTo(gray_old, CV_32FC1);

    std::vector<cv::Point> keypoints_old = tracker.extractKeypoints(gray_old, 150);

    for (int i = 1; i < images_paths.size(); i++){
        // Read image
        cv::Mat image_new = cv::imread(images_paths[i]);
        cv::imshow("Matches", image_new);
        cv::Mat gray_new;
        cv::cvtColor(image_new, gray_new, cv::COLOR_BGR2GRAY);
        gray_new.convertTo(gray_new, CV_32FC1);
        
        // Extract keypoints
        std::vector<cv::Point> keypoints_new = tracker.extractKeypoints(gray_new, 150);

        // Match keypoints 
        std::vector<int> matches = tracker.matchKeypoints(keypoints_new, gray_new, keypoints_old, gray_old);

        // Draw keypoints
        for (auto keypoint : keypoints_new){
            cv::circle(image_new, keypoint, 3, cv::Scalar(0, 0, 255), 3);
        }

        

        // Draw matches
        for (int i = 0; i < matches.size(); i++){
            if (matches[i] >= 0){
                cv::Point keypoint_new = keypoints_new[i];
                cv::Point keypoint_old = keypoints_old[matches[i]];

                cv::line(image_new, keypoint_new, keypoint_old, cv::Scalar(0,255,0), 3);
            }
        }

        // Update 
        gray_old = gray_new;
        keypoints_old = keypoints_new;

        cv::imshow("Matches", image_new);
        cv::waitKey(1);
    }

    return 0;
}