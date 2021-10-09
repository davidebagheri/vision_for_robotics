#include <opencv2/opencv.hpp>

cv::Mat from_vec_to_cv(std::vector<std::vector<float>> K){
    int rows = K.size();
    int cols = K[0].size();
    cv::Mat K_cv(rows, cols, CV_32F);

    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            K_cv.at<float>(i, j) = K[i][j];
        }
    }
    return K_cv;
}