#include "vo_odometry/initializer.h"

Initializer::Initializer(const cv::FileStorage& params){
    max_reproj_error_ = (float) params["initializer.max_reproj_error"];
}

void Initializer::initialize(Frame* init_frame, Frame* first_frame, Map* map, const Camera& camera){
    // Retrieve matched keypoints
    std::vector<cv::Point2f> first_frame_keypoints;
    for (int i = 0; i < init_frame->matches_.size(); i++){
        int first_frame_match = init_frame->matches_[i];
        first_frame_keypoints.push_back(first_frame->keypoints_[first_frame_match]);
    }

    // Find Essential Matrix through Ransac and 5pt algorithm
    std::cout << "[Initializer] Initializing with " << first_frame_keypoints.size() << " matches" << std::endl;
    cv::Mat match_inliers;
    cv::Mat E = cv::findEssentialMat(init_frame->keypoints_, first_frame_keypoints, camera.getCameraMatrix(), cv::RANSAC, 0.9999, max_reproj_error_, match_inliers);

    int ransac_5pt_algo_inliers_count = cv::countNonZero(match_inliers);
    std::cout << "[Initializer] Ransac-5pt algo -> Inliers: " << ransac_5pt_algo_inliers_count << "/" << match_inliers.rows << " (" << (float)ransac_5pt_algo_inliers_count * 100 / (float)match_inliers.rows << "%)" << std::endl;

    // Recover pose checking for each of the for possible solutions whether the points have positive depth
    cv::Mat R, t;
    cv::recoverPose( E, init_frame->keypoints_, first_frame_keypoints, camera.getCameraMatrix(), R, t, match_inliers);
    int recover_pose_inliers_count = cv::countNonZero(match_inliers);
    std::cout << "[Initializer] Recover pose -> Inliers: " << recover_pose_inliers_count << "/" << match_inliers.rows << " (" << (float)recover_pose_inliers_count * 100 / (float)match_inliers.rows << "%)" << std::endl;

    std::cout << "[Initializer] init R:" << std::endl; 
    std::cout << R << std::endl;
    std::cout << "[Initializer] init t:" << std::endl;
    std::cout << t << std::endl;

    init_frame->pose = cv::Affine3d(R, t);
    
    // Filter out the outliers
    init_frame->keypoints_ = utils::FilterVector(init_frame->keypoints_, match_inliers, nullptr);
    init_frame->matches_ = utils::FilterVector(init_frame->matches_, match_inliers, nullptr);

    // Gather the inliers keypoints of the first frame
    std::vector<cv::Point2f> first_inliers_keypoints;
    
    for (int i = 0; i < init_frame->keypoints_.size(); i++){
            int match = init_frame->matches_[i];
            first_inliers_keypoints.push_back(first_frame->keypoints_[match]);
    }

    // Triangulate inliers    
    cv::Mat firstProjMat = cv::Mat( camera.getCameraMatrix() ) * cv::Mat( first_frame->pose.inv().matrix ).rowRange(0, 3);
    cv::Mat initProjMat = cv::Mat( camera.getCameraMatrix() ) * cv::Mat( init_frame->pose.inv().matrix ).rowRange(0, 3);    

    cv::Mat points4D, points3D;
    cv::triangulatePoints(initProjMat, firstProjMat, init_frame->keypoints_, first_inliers_keypoints, points4D);
    cv::convertPointsFromHomogeneous(points4D.t(), points3D);
    points3D = points3D.reshape(1);     // Convert from an array of n Point3f to a matrix with size n x 3

    // Insert in the map
    init_frame->keypoints_map_id_ = map->registerPoints(points3D);

    std::cout << "[Initializer] Triangulated " << init_frame->keypoints_map_id_.size() << " points" << std::endl;
}