#ifndef UTILS_H_
#define ULTIS_H_

#pragma once

#include <opencv2/opencv.hpp>

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
    cv::Mat filterMat(const cv::Mat& input, const cv::Mat& mask){
        int n = cv::countNonZero(mask);

        cv::Mat output(n, input.cols, input.type());
        int i = 0;
        
        for (int j = 0; i < n; j++) {
            if (mask.at<uchar>(j))
            input.row(j).copyTo(output.row(i++));
        }

        return output;
    }

    template <typename T>
    std::vector<T> GetIndexedItems(const std::vector<T>& input, const std::vector<int>& indexes){
        std::vector<T> out;
        out.reserve(indexes.size());

        for (int i = 0; i < indexes.size(); i++)
            out.push_back(input[indexes[i]]);
        
        return out;
    }

    template <typename T>
    std::vector<T> GetIndexedItems(const std::vector<T>& input, const cv::Mat& indexes){
        std::vector<T> out;
        out.reserve(indexes.rows);

        for (int i = 0; i < indexes.rows; i++)
            out.push_back( input[indexes.at<int>(i) ]);
        
        return out;
    }
}

#endif