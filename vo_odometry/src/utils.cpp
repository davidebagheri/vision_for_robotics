#include "vo_odometry/utils.h"

namespace utils {
    
    template <typename T>
    std::vector<T> FilterVector(const std::vector<T>& input, const cv::Mat& mask, std::vector<int>* indexes){
        int n = cv::countNonZero(mask);
        
        std::vector<T> out;
        out.reserve(n);

        if (indexes != nullptr){
            indexes->clear();
            indexes->reserve(n);
        }

        for (int i = 0; i < input.size(); i++){
            if ( mask.at<uint8_t>(i) ){
                out.push_back(input[i]);
                
                if (indexes != nullptr)
                    indexes->push_back(i);
            }
        }

        return out;
    }

    template <typename T>
    std::vector<T> GetIndexedItems(const std::vector<T>& input, const std::vector<int>& indexes){
        std::vector<T> out;
        out.reserve(indexes.size());

        for (int i = 0; i < indexes.size(); i++)
            out.push_back(input[indexes[i]]);
        
        return out;
    }
}