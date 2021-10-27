#include <opencv2/opencv.hpp>

class Tracker{
public:
    Tracker(int patch_size = 10, float k_harris=0.001, float lambda=1000){
        patch_size_ = patch_size;
        k_harris_ = k_harris;
        lambda_ = lambda;
    }

    std::vector<cv::Point> extractKeypoints(const cv::Mat& image);

    std::vector<cv::Point> selectKeypoints(cv::Mat& harris, int nms_range, int n_keypoints);

    std::vector<int> matchKeypoints(const std::vector<cv::Point>& kp1, 
                                                    const cv::Mat& image1, 
                                                    const std::vector<cv::Point>& kp2, 
                                                    const cv::Mat& image2);
        
    float keypointDistanceSSD(const cv::Mat& kp1, const cv::Mat& kp2);

    //cv::Mat cornerHarris(const cv::Mat& image);

    //std::vector<cv::Mat> sobelXY(const cv::Mat& image);

    int getPatchSize(){
        return patch_size_;
    }

private:
    int patch_size_;
    float k_harris_;
    float lambda_;
};