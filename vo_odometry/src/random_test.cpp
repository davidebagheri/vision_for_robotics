#include <opencv2/opencv.hpp>


int main(){
    cv::Mat m = cv::Mat::eye(cv::Size(5,5), CV_8U);
    m.at<u_char>(0,1) = 5;

    cv::Scalar res =  cv::sum(m(cv::Range(1,5),cv::Range(1,5)));
    std::cout << res(0) << std::endl;
    return 0;
}