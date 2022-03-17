#include "vo_odometry/initializer.h"


Initializer::Initializer(const cv::FileStorage& params){
    bearing_angle_th_ = (int)params["initializer.bearing_angle_th"];
}

void Initializer::initialize(Frame* init_frame, 
                             Frame* first_frame,
                             const Camera& camera
                             ){
    // Retrieve matched keypoints
    std::vector<cv::Point2f> first_keypoints, init_keypoints;
    for (int i = 0; i < first_frame->matches_.size(); i++){
        int match = first_frame->matches_[i];
        if (match >= 0){
            first_keypoints.emplace_back(first_frame->keypoints_[i]);
            init_keypoints.emplace_back(init_frame->keypoints_[match]);
        }
    }

    // Find Essential Matrix through Ransac and 5pt algorithm
    cv::Mat match_inliers;
    cv::Mat E = cv::findEssentialMat(first_keypoints, init_keypoints, camera.getCameraMatrix(), cv::RANSAC, 0.9999, 1, match_inliers);

    // Recover pose checking for each of the for possible solutions whether the points have positive depth
    cv::Mat R, t;
    cv::recoverPose( E, first_keypoints, init_keypoints, camera.getCameraMatrix(), R, t, match_inliers);

    init_frame->R_ = convertRotationMatrix<float>(R.t());
    init_frame->t_ = convertTraslationVector<float>(-t);

    // Triangulate inliers  
    int n_inliers = 0;
    std::vector<cv::Point2f> init_kpts_inliers;
    std::vector<cv::Point3f> init_points_3d;

    // Precompute projection matrices
    cv::Matx34f firstProjMat = camera.getCameraMatrix() * getTransformFromRT<float>(first_frame->R_, first_frame->t_);
    cv::Matx34f initProjMat = camera.getCameraMatrix() * getTransformFromRT<float>(init_frame->R_, init_frame->t_);

    for (int i = 0; i < first_frame->keypoints_.size(); i++){
        if (match_inliers.at<uint8_t>(0, i) > 0){   // Check if inlier
            cv::Point2f& first_pixel = first_frame->keypoints_[i];
            cv::Point2f& init_pixel = init_frame->keypoints_[i];
            
            // Compute bearing vectors and the angle in between
            cv::Vec3f first_vec = first_frame->R_ * camera.camToWorld(first_pixel);
            cv::Vec3f init_vec = init_frame->R_ * camera.camToWorld(init_pixel);
            float angle = std::acos(first_vec.dot(init_vec) / (cv::norm(first_vec) * cv::norm(init_vec)));

            // Check if the angle between bearing vectors is enough  
            if (angle > bearing_angle_th_){
                // Triangulate
                cv::Mat point_4d;

                cv::triangulatePoints(initProjMat, 
                                      firstProjMat, 
                                      pixToVec<float>(init_pixel), 
                                      pixToVec<float>(first_pixel), 
                                      point_4d);
                
                // Normalize and store the result
                float x = point_4d.at<float>(0,0);
                float y = point_4d.at<float>(0,1);
                float z = point_4d.at<float>(0,2);
                float w = point_4d.at<float>(0,3);
                cv::Point3f pt3d(x/w, y/w, z/w);

                // Check for negative depth
                if (pt3d.z < 0) continue;  

                init_kpts_inliers.emplace_back(init_pixel);      
                init_points_3d.emplace_back(pt3d);   

                first_frame->matches_[i] = n_inliers;
                n_inliers++;
            } else {
                first_frame->matches_[i] = -1;  // Remove match
            }
        } else {
            first_frame->matches_[i] = -1;  // Remove match
        }
    }

    std::cout << "N inliers: " << n_inliers << std::endl;

    init_frame->keypoints_ = std::move(init_kpts_inliers);
    init_frame->matches_ = std::vector(n_inliers, -1);
    init_frame->points_3d_ = std::move(init_points_3d);
}