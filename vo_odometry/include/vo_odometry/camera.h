#ifndef PARAMS_H
#define PARAMS_H

#include <opencv2/opencv.hpp>


class Camera{
    public:
        Camera(const cv::FileStorage& params){
            // TODO: make it parametric
            K = cv::Matx33f(7.188560000000e+02, 0.0, 6.071928000000e+02,
                            0.0, 7.188560000000e+02, 1.852157000000e+02,
                            0.0, 0.0, 1.0);
            K_inv = cv::Matx33f(1 / K(0,0), 0, -K(0,2) / K(0,0),
                                0, 1 / K(1,1), -K(1,2) / K(1,1),
                                0, 0, 1);
        }

        const cv::Matx33f& getCameraMatrix() const { return K; }

        cv::Vec3f camToWorld(const cv::Point2f& pixel) const {
            cv::Vec3f pixel_h = {pixel.x, pixel.y, 1};
            return K_inv * pixel_h;
        }

        cv::Point2f worldToCam(const cv::Vec3f& point, const cv::Matx33f& R, const cv::Vec3f& t) const {
            cv::Vec3f pixel = K * (R * point + t);
            return cv::Point2f( pixel[0] / pixel[2], pixel[1] / pixel[2] );
        }

    private:
        cv::Matx33f K;
        cv::Matx33f K_inv;
};

#endif