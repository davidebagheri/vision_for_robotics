#include <iostream>
#include "data.h"
#include "utils.h"

int main(int argc, char** argv){
	std::string path_to_undistorted = "../data/images_undistorted";
	std::string path_to_K = "../data/K.txt";
	std::string path_to_D = "../data/D.txt";
	std::string path_to_csv = "../data/poses.txt";

	// Get data
	std::vector<std::string> undistorted_imgs_path = get_images_path(path_to_undistorted);
	std::vector<std::vector<float>> K = read_csv<float>(path_to_K);
	std::vector<std::vector<float>> D = read_csv<float>(path_to_D);
	cv::Mat undist_img = cv::imread(undistorted_imgs_path[0]);

	// Transform data to cv
	cv::Mat K_cv = from_vec_to_cv_mat(K);
	cv::Mat D_cv = from_vec_to_cv_mat(D);	

	// Read the first pose
	std::vector<float> first_pose = read_csv_line<float>(path_to_csv, 0);

	// Get points to project
	std::vector<std::vector<float>> point_list;
	std::vector<cv::Vec2f> point_list_proj;
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
		cv::Vec2f proj = projectPoint(point, K_cv, first_pose);
		point_list_proj.emplace_back(proj);
	}

	// Visualize 
	// points
	for (const cv::Vec2f& point_proj : point_list_proj){
		cv::circle(undist_img, cv::Point2i((int) point_proj[0],(int) point_proj[1]), 3, cv::Scalar(0, 0, 255), 3);
	}
	// sides
	cv::line(undist_img, cv::Point2i((int)point_list_proj[0][0], (int)point_list_proj[0][1]),
						 cv::Point2i((int)point_list_proj[1][0], (int)point_list_proj[1][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[1][0], (int)point_list_proj[1][1]),
						 cv::Point2i((int)point_list_proj[2][0], (int)point_list_proj[2][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[2][0], (int)point_list_proj[2][1]),
						 cv::Point2i((int)point_list_proj[3][0], (int)point_list_proj[3][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[3][0], (int)point_list_proj[3][1]),
						 cv::Point2i((int)point_list_proj[0][0], (int)point_list_proj[0][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[0][0], (int)point_list_proj[0][1]),
						 cv::Point2i((int)point_list_proj[4][0], (int)point_list_proj[4][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[1][0], (int)point_list_proj[1][1]),
						 cv::Point2i((int)point_list_proj[5][0], (int)point_list_proj[5][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[2][0], (int)point_list_proj[2][1]),
						 cv::Point2i((int)point_list_proj[6][0], (int)point_list_proj[6][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[3][0], (int)point_list_proj[3][1]),
						 cv::Point2i((int)point_list_proj[7][0], (int)point_list_proj[7][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[4][0], (int)point_list_proj[4][1]),
						 cv::Point2i((int)point_list_proj[5][0], (int)point_list_proj[5][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[5][0], (int)point_list_proj[5][1]),
						 cv::Point2i((int)point_list_proj[6][0], (int)point_list_proj[6][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[6][0], (int)point_list_proj[6][1]),
						 cv::Point2i((int)point_list_proj[7][0], (int)point_list_proj[7][1]),
						 cv::Scalar(0,0,255),
						 2
	);
	cv::line(undist_img, cv::Point2i((int)point_list_proj[4][0], (int)point_list_proj[4][1]),
						 cv::Point2i((int)point_list_proj[7][0], (int)point_list_proj[7][1]),
						 cv::Scalar(0,0,255),
						 2
	);

	cv::imshow("Projected points", undist_img);
	cv::waitKey(0);

	return 0;
}
