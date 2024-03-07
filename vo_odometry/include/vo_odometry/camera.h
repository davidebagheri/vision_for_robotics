#ifndef PARAMS_H
#define PARAMS_H

#include <opencv2/opencv.hpp>


class Camera{
    public:
        Camera(const cv::FileStorage& params){
            fx = (float)params["camera.fx"];
            fy = (float)params["camera.fy"];
            u0 = (float)params["camera.u0"];
            v0 = (float)params["camera.v0"];
            K = cv::Matx33f(fx, 0.0, u0, 0.0, fy, v0, 0.0, 0.0, 1.0);
            K_inv = cv::Matx33f(1 / fx, 0, -u0 / fx, 0, 1 / fy, -v0 / fy, 0, 0, 1);
        }

        const cv::Matx33f& getCameraMatrix() const {
            return K; 
        }

        cv::Vec3f camToWorld(const cv::Point2f& pixel) const {
            cv::Vec3f pixel_h = {pixel.x, pixel.y, 1};
            return K_inv * pixel_h;
        }

        cv::Point2f worldToCam(const cv::Vec3f& point, const cv::Matx33f& R, const cv::Vec3f& t) const {
            cv::Vec3f pixel = K * (R * point + t);
            return cv::Point2f( pixel[0] / pixel[2], pixel[1] / pixel[2] );
        }

    private:
        float fx;
        float fy;
        float u0;
        float v0;
        cv::Matx33f K;
        cv::Matx33f K_inv;
};

#endif