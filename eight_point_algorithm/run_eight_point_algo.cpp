#include <iostream>
#include "eight_point_algorithm/eight_point_algorithm.h"
#include "utils/conversion.h"
#include "utils/data.h"

int main(int argc, char** argv){
    /*if (argc != 2){
        std::cout << "Data path must be passed as argument" << std::endl;
        return -1;
    }

    std::string path2data = std::string(argv[1]);
    std::string path2matches1 = path2data + "/matches0001.txt";
    std::string path2matches2 = path2data + "/matches0002.txt";

    // Read matches
    std::vector<std::vector<float>> matches_1 = read_csv<float>(path2matches1, ' ');
    std::vector<std::vector<float>> matches_2 = read_csv<float>(path2matches2, ' ');

    std::vector<std::pair<cv::Point2f, cv::Point2f>> matches;

    for (int i = 0; i < matches_1[0].size(); i++){
        matches.emplace_back(std::pair(
            cv::Point2f(matches_1[0][i], matches_1[1][i]),
            cv::Point2f(matches_2[0][i], matches_2[1][i])));
    }

    EightPointAlgorithm epa;

    cv::Matx33f F = epa.computeFoundamentalMatrix(matches);

    std::cout << "Geom error " << epa.computeGeometricError(matches, F) << std::endl;
    std::cout << "Alg error " << epa.computeAlgebraicError(matches, F) << std::endl;*/
}