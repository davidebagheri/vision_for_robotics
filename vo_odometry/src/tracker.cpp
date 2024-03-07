#include "vo_odometry/tracker.h"

std::vector<cv::Point2f> Tracker::extractKeypoints(const Frame& frame, int max_n_keypoints, float quality_level, bool use_mask) const {
    cv::Mat mask(frame.img_.size(), CV_8U, cv::Scalar::all(255));
    
    if (use_mask){
        for (int i = 0; i < frame.keypoints_.size(); i++) {
            const cv::Point2f& kp = frame.keypoints_[i];
            if (kp.x < 5 || kp.y < 5 || kp.x > frame.img_.cols - 5 || kp.y > frame.img_.rows - 5) continue;
            mask(cv::Rect(kp - cv::Point2f(4, 4), cv::Size(9, 9))) = 0;
        }

        for (int i = 0; i < frame.candidate_keypoints_.size(); i++) {
            const cv::Point2f& kp = frame.candidate_keypoints_[i];
            if (kp.x < 5 || kp.y < 5 || kp.x > frame.img_.cols - 5 || kp.y > frame.img_.rows - 5) continue;

            mask(cv::Rect(kp - cv::Point2f(4, 4), cv::Size(9, 9))) = 0;
        }
    }

    std::vector<cv::Point2f> keypoints;
    cv::goodFeaturesToTrack(frame.img_, keypoints, max_n_keypoints, quality_level, 9, mask);

    std::cout << "[Tracker] Extracted " << keypoints.size() << " keypoints" << std::endl;

    return keypoints;
}

void Tracker::trackKeypoints(const Frame& previous_frame, Frame* next_frame) const {    
    // Track with Lukas Kanade algorithm the keypoints from the previous frame
    std::vector<cv::Point2f> res_kpts;
    cv::Mat status;
    std::vector<float> err;
    
    cv::TermCriteria criteria = cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 2000, 0.01);

    cv::calcOpticalFlowPyrLK(previous_frame.img_, next_frame->img_, previous_frame.keypoints_, res_kpts, status, err, cv::Size(21, 21), 5, criteria);

    // Filter out outliers
    next_frame->keypoints_ = utils::FilterVector(res_kpts, status, &next_frame->matches_);

    // Propagate the keypoints map id to the next frame
    next_frame->keypoints_map_id_ = utils::FilterVector(previous_frame.keypoints_map_id_, status, nullptr);

    std::cout << "[Tracker] Tracked " << next_frame->keypoints_.size() << "/" << previous_frame.keypoints_.size() << " keypoints" << std::endl;

    // Track the candidate keypoints from the previous frame
    if (previous_frame.candidate_keypoints_.size() > 0){
        std::vector<cv::Point2f> res_kpts;
        cv::Mat status;
        std::vector<float> err;

        cv::calcOpticalFlowPyrLK(previous_frame.img_, next_frame->img_, previous_frame.candidate_keypoints_, res_kpts, status, err);

        // Filter out outliers and keypoints with high error
        next_frame->candidate_keypoints_ = utils::FilterVector(res_kpts, status, &next_frame->candidate_keypoints_matches_);

        std::cout << "[Tracker] Tracked " << next_frame->candidate_keypoints_.size() << "/" << previous_frame.candidate_keypoints_.size() << " candidate keypoints" << std::endl;
    }
}

bool Tracker::estimatePose(const Frame& previous_frame, Frame* cur_frame, const Map& map, const Camera& cam) const {
    // Gather the matched 3d points
    std::vector<cv::Point3f> points_3d = map.getPoints(cur_frame->keypoints_map_id_);

    // Estimate pose through PnP
    assert(points_3d.size() == cur_frame->keypoints_.size());
    std::cout << "[Tracker] Solving PnP with " << points_3d.size() << " points" << std::endl;

    cv::Mat R_CW, t_CW;
    cv::Mat inliers;

    cv::solvePnPRansac(points_3d, cur_frame->keypoints_, cam.getCameraMatrix(), cv::noArray(),
                       R_CW, t_CW, false, 2000, 2.5, 0.9999, inliers);
        
    cur_frame->pose = cv::Affine3d(R_CW, t_CW).inv(); 

    std::cout << "Current pose:" << std::endl << "R: " << std::endl << cur_frame->pose.rotation() << std::endl << cur_frame->pose.translation() << std::endl;

    // Filter out the outliers
    cur_frame->keypoints_ = utils::GetIndexedItems(cur_frame->keypoints_, inliers);
    cur_frame->matches_ = utils::GetIndexedItems(cur_frame->matches_, inliers);
    cur_frame->keypoints_map_id_ = utils::GetIndexedItems(cur_frame->keypoints_map_id_, inliers);

    std::cout << "[Tracker] Estimated pose with PnPRansac and " << cur_frame->keypoints_.size() << " inliers" << std::endl;
    cur_frame->n_keypoints_used_for_pose_estimation_ = cur_frame->keypoints_.size();

    return true;
}


void Tracker::triangulateUnmatchedKeypoints(Frame* previous_frame, Frame* cur_frame,  Map* map, const Camera& camera) const {
    if (previous_frame->candidate_keypoints_.size() == 0) return;
    
    std::vector<cv::Point2f> previous_keypoints = utils::GetIndexedItems(previous_frame->candidate_keypoints_, cur_frame->candidate_keypoints_matches_);

    // Triangulate keypoints
    cv::Mat firstProjMat = cv::Mat( camera.getCameraMatrix() ) * cv::Mat( previous_frame->pose.inv().matrix ).rowRange(0, 3);
    cv::Mat initProjMat = cv::Mat( camera.getCameraMatrix() ) * cv::Mat( cur_frame->pose.inv().matrix ).rowRange(0, 3);    

    cv::Mat points4D;
    cv::triangulatePoints(initProjMat, firstProjMat, cur_frame->candidate_keypoints_, previous_keypoints, points4D);
    cv::Mat points3D;
    cv::convertPointsFromHomogeneous(points4D.t(), points3D);
    points3D = points3D.reshape(1);     // Convert from an array of n Point3f to a matrix with size n x 3

    // Filter out the points too close or too far from the camera position
    int n = points3D.rows;
    cv::Mat inliers = cv::Mat::ones(n, 1, CV_8U);

    for (int i = 0; i < n; i++){
        cv::Point3f pt_C = cur_frame->pose.inv() * points3D.at<cv::Point3f>(i); // Point in camera frame
        if (pt_C.z < min_depth_ or pt_C.z > max_depth_) {
            inliers.at<uint8_t>(i) = 0;
        }
    }
    
    cur_frame->candidate_keypoints_ = utils::FilterVector(cur_frame->candidate_keypoints_, inliers, nullptr);
    cur_frame->candidate_keypoints_matches_ = utils::FilterVector(cur_frame->candidate_keypoints_matches_, inliers, nullptr);
    points3D = utils::filterMat<cv::Point3f>(points3D, inliers);  

    // Add new points to the map
    std::vector<int> candidate_keypoints_map_id = map->registerPoints(points3D);
    
    // Add the traingulated candidate keypoints to the keypoints vectors 
    for (int i = 0; i < cur_frame->candidate_keypoints_.size(); i++){
        cur_frame->keypoints_.push_back( cur_frame->candidate_keypoints_[i] );
        cur_frame->keypoints_map_id_.push_back( candidate_keypoints_map_id[i] );

        // Update matches with the previous frame
        int candidate_match = cur_frame->candidate_keypoints_matches_[i]; 
        previous_frame->keypoints_.push_back( previous_frame->candidate_keypoints_[candidate_match] );
        cur_frame->matches_.push_back(previous_frame->keypoints_.size() - 1);
    }

    cur_frame->candidate_keypoints_.clear();
    cur_frame->candidate_keypoints_matches_.clear();

    std::cout << "[Tracker] Added " << candidate_keypoints_map_id.size() << " triangulated keypoints to the current frame" << std::endl;
}

void Tracker::addNewKeypoints(Frame* frame, const std::vector<cv::Point2f>& new_kpts) const {
    for (int i = 0; i < new_kpts.size(); i++)
        frame->candidate_keypoints_.push_back(new_kpts[i]);

    std::cout << "[Tracker] Added " << new_kpts.size() << " candidate keypoints" << std::endl;
    
} 