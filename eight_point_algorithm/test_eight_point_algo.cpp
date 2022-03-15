#include <iostream>
#include <ctime>
#include <random>
#include "eight_point_algorithm/eight_point_algorithm.h"


int main(int argc, char** argv){
    srand(42);
    int n_points = 40;
    std::vector<cv::Vec3d> pix_1, pix_2, noisy_pix_1, noisy_pix_2;

    std::default_random_engine generator;
    std::normal_distribution<double> noise_distribution(0, 1e-1);
    cv::Matx34d proj_1(500, 0, 320, 0, 0, 500, 240, 0, 0, 0, 1, 0);
    cv::Matx34d proj_2(500, 0, 320, -100, 0, 500, 240, 0, 0, 0, 1, 0);

    // Generate 3d points and compute matches
    for (int i = 0; i < n_points; i++){
        cv::Vec4d h_point(float(rand()) / RAND_MAX,
                        float(rand()) / RAND_MAX,
                        float(rand()) / RAND_MAX * 5 + 10,
                        1.0);
        cv::Vec3d pix1 = proj_1 * h_point;
        cv::Vec3d pix2 = proj_2 * h_point;
        pix1 = pix1 / pix1[2];
        pix2 = pix2 / pix2[2];
        cv::Vec3d noisy_pix1 = pix1 + cv::Vec3d(noise_distribution(generator),
                                                noise_distribution(generator),
                                                noise_distribution(generator));
        cv::Vec3d noisy_pix2 = pix2 + cv::Vec3d(noise_distribution(generator),
                                        noise_distribution(generator),
                                        noise_distribution(generator));

        pix_1.emplace_back(pix1);
        pix_2.emplace_back(pix2);
        noisy_pix_1.emplace_back(noisy_pix1);
        noisy_pix_2.emplace_back(noisy_pix2);
    }

    EightPointAlgorithm epa;


    std::cout << "Noise free case:" << std::endl;
    cv::Matx33d F = epa.computeFoundamentalMatrix(pix_1, pix_2);
    std::cout << "Algebraic error: " << epa.computeAlgebraicError(pix_1, pix_2, F) << std::endl;
    std::cout << "Geometric error: " << epa.computeGeometricError(pix_1, pix_2, F) << std::endl;

    std::cout << "With Noise case:" << std::endl;
    cv::Matx33d F_noise = epa.computeFoundamentalMatrix(noisy_pix_1, noisy_pix_2);
    std::cout << "Algebraic error: " << epa.computeAlgebraicError(pix_1, pix_2, F_noise) << std::endl;
    std::cout << "Geometric error: " << epa.computeGeometricError(pix_1, pix_2, F_noise) << std::endl;

    std::cout << "Normalized case:" << std::endl;
    cv::Matx33d F_norm = epa.computeFoundamentalMatrixNormalized(noisy_pix_1, noisy_pix_2);
    std::cout << "Algebraic error: " << epa.computeAlgebraicError(pix_1, pix_2, F_norm) << std::endl;
    std::cout << "Geometric error: " << epa.computeGeometricError(pix_1, pix_2, F_norm) << std::endl;
}
