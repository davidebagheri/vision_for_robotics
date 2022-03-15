#include <opencv2/opencv.hpp>


int main(){
    cv::Matx33f K = cv::Matx33f(7.188560000000e+02, 0.0, 6.071928000000e+02,
                            0.0, 7.188560000000e+02, 1.852157000000e+02,
                            0.0, 0.0, 1.0);
       
    cv::Matx proj1 = cv::Matx34f(1, 0, 0, 0,
                                 0, 1, 0, 0,
                                 0, 0, 1, 0);

    cv::Matx proj2 = cv::Matx34f(1, 0, 0, 0.01,
                                 0, 1, 0, 0,
                                 0, 0, 1, 0);

    cv::Vec4f point = {0, 5, 3, 1};

    cv::Vec3f pt1 = proj1 * point;
    cv::Vec3f pt2 = proj2 * point;

    std::cout << pt1 / 3 << pt2 / 3 << std::endl;

    cv::Point2f pixel1 = cv::Point2f(pt1[0]/pt1[2], pt1[1]/pt1[2]);
    cv::Point2f pixel2 = cv::Point2f(pt2[0]/pt2[2], pt2[1]/pt2[2]);
    std::cout << pixel1 << pixel2 << std::endl;
    cv::Mat res;
    cv::Vec2f p1 = {pixel1.x, pixel1.y};
    cv::triangulatePoints(proj1, proj2, p1, std::vector<cv::Point2f>({pixel2}), res);

    float x = res.at<float>(0,0);
    float y = res.at<float>(0,1);
    float z = res.at<float>(0,2);
    float w = res.at<float>(0,3);

    std::cout << "x: " << x/w << " y: " << y/w << " z: " << z/w << std::endl;

    return 0;
}