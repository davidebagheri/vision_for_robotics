#include <iostream>
#include "data.h"
#include "utils.h"

int main(int argc, char** argv){
	std::string path_to_undistorted = "../data/images_undistorted";
	std::string path_to_K = "../data/K.txt";
	std::string path_to_D = "../data/D.txt";

	// Get data
	std::vector<std::string> undistorted_imgs_path = get_images_path(path_to_undistorted);
	std::vector<std::vector<float>> K = read_csv<float>(path_to_K);
	std::vector<std::vector<float>> D = read_csv<float>(path_to_D);

	// Transform data to cv
	cv::Mat K_cv = from_vec_to_cv_mat(K);
	cv::Mat D_cv = from_vec_to_cv_mat(D);	

	return 0;
}
