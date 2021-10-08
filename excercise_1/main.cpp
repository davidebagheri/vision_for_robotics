#include <data.h>
#include <iostream>

int main(int argc, char** argv){
	std::string path_to_undistorted = "../data/images_undistorted";
	
	std::vector<std::string> undistorted_imgs_path = get_images_path(path_to_undistorted);

	for (auto i : undistorted_imgs_path){
		std::cout << i << std::endl;
	}
}
