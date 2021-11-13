#include <iostream>
#include "utils/data.h"
#include "utils/conversion.h"
#include "pinhole_camera/camera_operations.h"

int main(int argc, char** argv){
	if (argc != 2){
		std::cout << "Provide path to data as param!" << std::endl;
		return -1;
	}
	std::string path_to_data(argv[1]);
	std::string path_to_undistorted = path_to_data + "/images_undistorted";
	std::string path_to_K = path_to_data + "/K.txt";
	std::string path_to_csv = path_to_data + "/poses.txt";

	// Get data
	std::vector<std::string> undistorted_imgs_path = get_images_path(path_to_undistorted);
	std::vector<std::vector<float>> K = read_csv<float>(path_to_K);
	cv::Mat undist_img = cv::imread(undistorted_imgs_path[0]);

	// Transform data to cv
	cv::Mat K_cv = from_vec_to_cv_mat(K);

	// Read the first pose
	std::vector<float> first_pose = read_csv_line<float>(path_to_csv, 0);

	// Get points to project
	std::vector<std::vector<float>> point_list;
	std::vector<cv::Point2f> point_list_proj;
	float square_size = 0.04;
	point_list = {{3 * square_size, 3 * square_size, 0}, 
				  {5 * square_size, 3 * square_size, 0},
				  {5 * square_size, 5 * square_size, 0},
				  {3 * square_size, 5 * square_size, 0},
				  {3 * square_size, 3 * square_size, -2 * square_size}, 
				  {5 * square_size, 3 * square_size, -2 * square_size},
				  {5 * square_size, 5 * square_size, -2 * square_size},
				  {3 * square_size, 5 * square_size, -2 * square_size}
				  };	

	// Project points and visualize
	for (std::vector<float>& point : point_list){
		cv::Point2f proj = projectPoint(point, K_cv, first_pose);
		point_list_proj.emplace_back(proj);
	}

	// Visualize 
	// points
	for (const cv::Point2i& point_proj : point_list_proj){
		cv::circle(undist_img, point_proj, 3, cv::Scalar(0, 0, 255), 3);
	}
	// sides
	cv::line(undist_img, point_list_proj[0],
						 point_list_proj[1],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[1],
						 point_list_proj[2],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[2],
						 point_list_proj[3],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[3],
						 point_list_proj[0],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[0], 
						 point_list_proj[4],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[1],
						 point_list_proj[5],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[2],
						 point_list_proj[6],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[3],
						 point_list_proj[7],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[4],
						 point_list_proj[5],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[5],
						 point_list_proj[6],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[6],
						 point_list_proj[7],
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, point_list_proj[4],
						 point_list_proj[7],
						 cv::Scalar(0,0,255),
						 2
	);

	cv::imshow("Projected points", undist_img);
	cv::waitKey(0);

	return 0;
}
