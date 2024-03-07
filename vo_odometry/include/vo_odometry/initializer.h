#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "vo_odometry/frame.h"
#include "vo_odometry/camera.h"
#include "vo_odometry/utils.h"
#include "vo_odometry/map.h"

class Initializer {
public:
    Initializer(const cv::FileStorage& params);

    void initialize(Frame* init_frame,
                    Frame* first_frame,
                    Map* map,
                    const Camera& camera);
private:
    float max_reproj_error_ = 1.;
};

#endif
