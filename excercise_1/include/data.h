#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <glob.h>
#include <vector>

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

template <class T>
std::vector<std::vector<T>> read_csv(const std::string& path_to_csv, char delimiter = ' ')
{
    // Open file
    std::ifstream file(path_to_csv);
    if (!file.is_open()) throw std::runtime_error("Could not open the file " + path_to_csv);

    std::string line;
    T val;
    std::vector<std::vector<T>> data;

    // Read line by line
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::vector<T> values;
        
        while (ss >> val)
        {
            values.emplace_back(val);
            // If the next token is a comma, ignore it and move on
            if (ss.peek() == delimiter) ss.ignore();
        }
        data.emplace_back(values);
    }
    return data;
} 
