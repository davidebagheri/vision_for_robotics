#include <iostream>
#include "utils/data.h"
#include "utils/conversion.h"
#include "excercise_1/camera_operations.h"

int main(int argc, char** argv){
	if (argc != 2){
		std::cout << "Provide path to data as param!" << std::endl;
		return -1;
	}
	std::string path_to_data(argv[1]);
	std::string path_to_undistorted = path_to_data + "/images_undistorted";
	std::string path_to_distorted = path_to_data + "/images";
	std::string path_to_D = path_to_data + "/D.txt";

	// Get data
	std::vector<std::string> undistorted_imgs_path = get_images_path(path_to_undistorted);
	std::vector<std::string> distorted_imgs_path = get_images_path(path_to_distorted);
	std::vector<std::vector<float>> D = read_csv<float>(path_to_D);
	//cv::Mat undist_img = cv::imread(undistorted_imgs_path[0]);
	cv::Mat dist_img = cv::imread(distorted_imgs_path[0], cv::IMREAD_GRAYSCALE);

	std::vector<float> D_coeff = D[0];

	cv::Mat undist_img = undistortImage(dist_img, D_coeff);

	cv::imshow("Undistorted image", undist_img);
	cv::waitKey(0);

	return 0;
}
