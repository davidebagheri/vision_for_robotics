#include "vo_odometry/map.h"

std::vector<int> Map::registerPoints(const cv::Mat& new_points){

    assert(new_points.cols == 3); // new_points must be a n x 3 matrix

    std::vector<int> indexes(new_points.rows);

    int next_idx = points_3d_.size();

    for (int i = 0; i < new_points.rows; i++){
        points_3d_.push_back( new_points.at<cv::Point3f>(i) );
        indexes[i] = next_idx;
        next_idx++;
    }

    return indexes;
}

std::vector<cv::Point3f> Map::getPoints(const std::vector<int>& indexes) const {
    return utils::GetIndexedItems(points_3d_, indexes);
}
