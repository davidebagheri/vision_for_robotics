#ifndef CONVERTER_H
#define CONVERTER_H

#include <opencv2/opencv.hpp>


template <typename T>
cv::Matx<T, 3, 3> convertRotationMatrix(const cv::Mat& R){
    cv::Matx<T, 3, 3> conv_R;

    switch (R.type())
    {
        case CV_32F:
        {
            conv_R = cv::Matx<T, 3, 3>(R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2),
                                       R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), 
                                       R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2));
            break;
        }
        case CV_64F:
        {
            conv_R = cv::Matx<T, 3, 3>(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                       R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), 
                                       R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
            break;
        }
        default:
        {
            std::cerr << "No conversion type implemented!" << std::endl;
            break;
        }
    }

    return conv_R;
}

template <typename T>
cv::Vec<T, 3> convertTraslationVector(const cv::Mat& t){
    cv::Vec<T, 3> conv_t;
   
    switch (t.type())
    {
        case CV_32F:
        {
            conv_t = cv::Vec<T, 3>(t.at<float>(0), t.at<float>(1), t.at<float>(2));
            break;
        }
        case 6:
        {
            conv_t = cv::Vec<T, 3>(t.at<double>(0), t.at<double>(1), t.at<double>(2));
            break;
        }
        default:
        {
            std::cerr << "No conversion type implemented!" << std::endl;
            break;
        }
    }

    return conv_t;
}

template <typename T>
cv::Matx<T, 3, 4> getTransformFromRT(const cv::Matx<T, 3, 3>& R, const cv::Vec<T, 3>& t){
    cv::Matx<T, 3, 4> out;

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            out(i,j) = R(i,j);
        }
    }

    for (int i = 0; i < 3; i++){
        out(i,3) = t(i);
    }

    return out;
}

template <typename T>
cv::Vec<T, 2> pixToVec(const cv::Point_<T> pixel){
    return cv::Vec<T, 2>(pixel.x, pixel.y);
}

#endif