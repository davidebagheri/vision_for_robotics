#include "data.h"
#include <glob.h>
#include <iostream>

std::vector<std::string> get_images_path(const std::string& path_to_img_folder){
    std::vector<std::string> ret;
    std::string imgs_path_pattern = path_to_img_folder + "/*.jpg";

    glob_t glob_result;
    glob(imgs_path_pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
    
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        ret.push_back(std::string(glob_result.gl_pathv[i]));
    }

    globfree(&glob_result);
    return ret;
}