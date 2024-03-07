#ifndef MAP_H
#define MAP_H

#include "vo_odometry/utils.h"

class Map {
public:
    std::vector<int> registerPoints(const cv::Mat& new_points);

    std::vector<cv::Point3f> getPoints(const std::vector<int>& indexes)  const;

private:
    std::vector<cv::Point3f> points_3d_;
};

#endif