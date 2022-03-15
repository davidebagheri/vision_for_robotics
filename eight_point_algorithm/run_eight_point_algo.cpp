#include <iostream>
#include "eight_point_algorithm/eight_point_algorithm.h"
#include "utils/conversion.h"
#include "utils/data.h"

int main(int argc, char** argv){
    if (argc != 2){
        std::cout << "Data path must be passed as argument" << std::endl;
        return -1;
    }

    std::string path2data = std::string(argv[1]);
    std::string path2matches1 = path2data + "/matches0001.txt";
    std::string path2matches2 = path2data + "/matches0002.txt";

    // Read matches
    std::vector<std::vector<float>> matches_1 = read_csv<float>(path2matches1, ' ');
    std::vector<std::vector<float>> matches_2 = read_csv<float>(path2matches2, ' ');
    std::vector<cv::Point2f> points_1, points_2;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> matches;

    for (int i = 0; i < matches_1[0].size(); i++){
        points_1.emplace_back(cv::Point2f(matches_1[0][i], matches_1[1][i]));
        points_2.emplace_back(cv::Point2f(matches_2[0][i], matches_2[1][i]));
    }

    // Load images
    cv::Mat img1 = cv::imread(path2data + "/0001.jpg");
    cv::Mat img2 = cv::imread(path2data + "/0002.jpg");

    // Camera matrices 
    cv::Matx33d K1(1379.74, 0, 760.35, 0, 1382.08, 503.41, 0, 0, 1);
    cv::Matx33d K2 = K1;

    EightPointAlgorithm epa;

    // Estimate essential matrix with 8-point algorithm
    cv::Matx33d E = epa.estimateEssentialMatrix(points_1, points_2, K1, K2);

    // Obtain extrinsic parameters from E
    /*std::vector<cv::Matx34d> possible_T_12 = epa.decomposeEssentialMatrix(E); 
    
    // Extract the relative camera positions from the essential matrix
    cv::Matx34d T_12 = epa.disambiguateRelativePose(possible_T_12, K1, K2, points_1, points_2);

    // Triangulate a point cloud using the final transformation
    cv::Matx34f M1;
    cv::Matx34f M2 = K2 * T_12;
    for (int i = 0; i < points_1.size(); i++){
        cv::Point3f point = epa.triangulatePoints(points_1[i], points_2[i], M1, M2);
    }*/

}