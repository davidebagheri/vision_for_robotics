#include <iostream>

#include "simple_keypoint_tracker/tracker.h"
#include "utils/data.h"

int main(int argc, char** argv){
    if (argc != 2){
        std::cout << "Data path must be passed as argument" << std::endl;
    }

    std::string data_path(argv[1]);
    std::vector<std::string> images_paths = get_images_path(data_path, "/*.png");

    Tracker tracker(16, 0.01, 3);

    cv::Mat gray_old = cv::imread(images_paths[0], 0);
    gray_old.convertTo(gray_old, CV_32FC1);

    std::vector<cv::Point> keypoints_old = tracker.extractKeypoints(gray_old);

    for (int i = 1; i < images_paths.size(); i++){
        // Read image
        cv::Mat image_new = cv::imread(images_paths[i]);
        cv::Mat gray_new;
        cv::cvtColor(image_new, gray_new, CV_BGR2GRAY);
        gray_new.convertTo(gray_new, CV_32FC1);
        
        // Extract keypoints
        std::vector<cv::Point> keypoints_new = tracker.extractKeypoints(gray_new);

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
        cv::waitKey(2);

    }

    /*for (auto image_path : images_paths){
        cv::Mat image = cv::imread(image_path);
        cv::Mat gray;
        cv::cvtColor(image, gray, CV_BGR2GRAY);

        Tracker tracker(10, 0.01);
        std::vector<cv::Point> keypoints = tracker.extractKeypoints(gray);

        for (auto keypoint : keypoints){
            cv::circle(image, keypoint, 3, cv::Scalar(0, 0, 255), 3);
        }

        cv::imshow("keypoints", image);
        cv::waitKey(2);
    }*/

    /*float data_a[] = {1,2,3,4,5,6,7,8,9};
    cv::Mat a(3, 3, CV_32FC1, data_a);

    float data_b[] = {1,2,0,4,5,6,7,8,9};
    cv::Mat b(3, 3, CV_32FC1, data_b);

    cv::Mat diff = a(cv::Range(0,2), cv::Range(0,3)) - b(cv::Range(0,2), cv::Range(0,3));
    cv::Mat square = diff.mul(diff);
    std::cout << cv::sum(square)[0] << std::endl;
    */

    return 0;
}