#ifndef INITIALIZER_H
#define INITIALIZER_H


#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "vo_odometry/frame.h"
#include "vo_odometry/camera.h"
#include "vo_odometry/converter.h"

class Initializer {
public:
    Initializer(const cv::FileStorage& params);

    void initialize(Frame* init_frame,
                    Frame* first_frame,
                    const Camera& camera); 


    bool checkTriangPoint(const cv::Point3f& pt3d);

private:
    float bearing_angle_th_;
};

#endif
