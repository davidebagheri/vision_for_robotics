#include <opencv2/opencv.hpp>
#include "vo_odometry/initializer.h"
#include "utils/visualizer.h"


int main(){
    cv::Mat first_img = cv::imread("/home/dbagheri/Desktop/vision_for_robotics/vo_odometry/data/kitti/05/image_0/000001.png", cv::IMREAD_GRAYSCALE);
    cv::Mat cur_img = cv::imread("/home/dbagheri/Desktop/vision_for_robotics/vo_odometry/data/kitti/05/image_0/000003.png", cv::IMREAD_GRAYSCALE);

    cv::FileStorage params("/home/dbagheri/Desktop/vision_for_robotics/vo_odometry/cfg/kitty.yaml", cv::FileStorage::READ);
    FeatureMatcher feature_matcher(params);
    
    std::vector<cv::Point> p0_i = feature_matcher.extractKeypoints(first_img);
    std::vector<cv::Point2f> p0, p1;
    std::vector<uchar> status;
    std::vector<float> err;

    for (auto p : p0_i){
        p0.emplace_back(cv::Point2f(p.x, p.y));
    }

    cv::calcOpticalFlowPyrLK(first_img, cur_img, p0, p1, status, err);


    for (auto p : p0_i){
        cv::circle(cur_img, p, 2, cv::Scalar(255,0,0), 2);
    }
    /*for (int i = 0; i < p0_i.size(); i++){

        cv::circle(cur_img, cv::Point(p.x, p,y), 2, cv::Scalar(0,0,255), 2);
    }*/

    

    cv::imshow("a", cur_img);
    cv::waitKey();
    return 0;
}

