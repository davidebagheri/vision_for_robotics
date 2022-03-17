#include "vo_odometry/tracker.h"
#include "vo_odometry/converter.h"

std::vector<cv::Point2f> Tracker::extractKeypoints(const cv::Mat& image) const{
    // Compute harris score
    cv::Mat harris;
    cv::cornerHarris(image, harris, patch_radius_*2 + 1, 3, 0.04);

    // Select Keypoints
    std::vector<cv::Point2f> keypoints = selectKeypoints(harris, 10, n_keypoints_);

    return keypoints;
}

std::vector<cv::Point2f> Tracker::selectKeypoints(const cv::Mat& harris, 
                                                       int nms_range, 
                                                       int n_keypoints) const {
    std::vector<cv::Point2f> keypoints;

    for (int i = 0; i < n_keypoints; i++){
        // Find current max score
        cv::Point min_loc, max_loc;
        double min, max;
        cv::minMaxLoc(harris, &min, &max, &min_loc, &max_loc);

        // Select keypoints in a descrtiptable position
        if (max_loc.x > patch_radius_ and max_loc.x < harris.cols - patch_radius_ - 1
        and max_loc.y > patch_radius_ and max_loc.y < harris.rows - patch_radius_ - 1){
            keypoints.emplace_back(cv::Point2f(max_loc.x, max_loc.y));
        }

        // Set neighbouring pixels to zero
        int min_x = std::max(0, max_loc.x - nms_range);
        int max_x = std::min(harris.cols, max_loc.x + nms_range);
        int min_y = std::max(0, max_loc.y - nms_range);
        int max_y = std::min(harris.rows, max_loc.y + nms_range);
        harris(cv::Range(min_y, max_y), cv::Range(min_x, max_x)) = 0;
    }

    return keypoints;
}

void Tracker::extractKeypoints(Frame* frame) const {
    frame->keypoints_ = extractKeypoints(frame->img_);
    frame->matches_ = std::vector<int>(frame->keypoints_.size(), -1);
    frame->points_3d_ = std::vector<cv::Point3f>(frame->keypoints_.size(), cv::Point3f(INFINITY, INFINITY, INFINITY));
}

void Tracker::addNewKeypoints(Frame* frame) const {
    if (frame->keypoints_.size() >= n_max_kpts_) return;    // Enough keypoints

    // Store the keypoints positions in a binary image
    cv::Mat kpts_img = cv::Mat::zeros(frame->img_.size(), CV_8U);

    for (auto& kp : frame->keypoints_){
        kpts_img.at<uint8_t>(kp.y, kp.x) = 1;
    }

    // Extract new keypoints
    std::vector<cv::Point2f> new_kpts = extractKeypoints(frame->img_);

    // Check for vicinity keypoints
    for (auto& kp : new_kpts){
        cv::Mat patch = kpts_img(
            cv::Range(std::max(0, (int)kp.y - max_dist_new_kp_), std::min(kpts_img.rows, (int)kp.y + max_dist_new_kp_)),
            cv::Range(std::max(0, (int)kp.x - max_dist_new_kp_), std::min(kpts_img.cols, (int)kp.x + max_dist_new_kp_))
        );

        if (cv::sum(patch)(0) > 0) continue;    // There is already another keypoint close to the new computed one

        frame->keypoints_.emplace_back(kp);
        frame->matches_.emplace_back(-1);

        if (frame->keypoints_.size() >= n_max_kpts_) return;  // Max number of keypoints reached
    }
}

void Tracker::trackPoints(const cv::Mat& old_image,
                    const cv::Mat& new_image, 
                    const std::vector<cv::Point2f>& old_kpts, 
                    std::vector<cv::Point2f>& next_kpts,
                    std::vector<int>& matches){
    // Clear previous result
    next_kpts.clear();

    // Track with Lukas Kanade algorithm
    std::vector<cv::Point2f> res_kpts;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(old_image, new_image, old_kpts, res_kpts, status, err);

    // Filter out outliers and keypoints with high error
    int counter = 0;
    std::vector<int> good_matches(old_kpts.size(), -1); // Vector with -1 if no match found, otherwise the index of the new keypoint match

    for (int i = 0; i < res_kpts.size(); i++){
        if (status[i] and err[i] <= tracking_err_th_){
            next_kpts.emplace_back(res_kpts[i]);
            good_matches[i] = counter;
            counter++;
        }
    }

    matches = std::move(good_matches);
}


void Tracker::trackPoints(Frame& old_frame, Frame& new_frame){
    trackPoints(old_frame.img_, new_frame.img_, old_frame.keypoints_, new_frame.keypoints_, old_frame.matches_);
}
    
void Tracker::estimatePose(Frame& old_frame, Frame& new_frame, std::vector<int>& inlier_matches, const Camera& camera){
    int n_old_triang_pts = old_frame.points_3d_.size(); // Number of the points triangulated in the old frame

    // Estimate Pose with Pnp and Ransac
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> new_pixels;
    std::vector<int> match_new_old;
    
    for (int i = 0; i < n_old_triang_pts; i++){
        if (old_frame.matches_[i] > -1){
            pts3d.emplace_back(old_frame.points_3d_[i]);
            new_pixels.emplace_back(new_frame.keypoints_[old_frame.matches_[i]]);
            match_new_old.emplace_back(i);
        }
    }

    std::cout << "Pnp inliers are " << pts3d.size() << " keypoints!\n";

    cv::Mat Rvec, t, inliers;
    cv::solvePnPRansac(pts3d, new_pixels, camera.getCameraMatrix(), cv::noArray(), Rvec, t, false, 100, 40, 0.99, inliers);
    
    cv::Mat R;
    cv::Rodrigues(Rvec, R);
    
    new_frame.R_ = convertRotationMatrix<float>(R);
    new_frame.t_ = convertTraslationVector<float>(t);

    for (int i = 0; i < inliers.rows; i++){
        inlier_matches.emplace_back(match_new_old[inliers.at<int>(i,0)]);
    }
}

std::vector<cv::Point3f> Tracker::triangulateAddedKeypoints(Frame& old_frame, Frame& new_frame, const Camera& camera){
    std::vector<cv::Point3f> res;
    int n_old_triang_pts = old_frame.points_3d_.size(); // Number of the points triangulated in the old frame

    cv::Matx34f oldProjMat = camera.getCameraMatrix() * getTransformFromRT<float>(old_frame.R_, old_frame.t_);
    cv::Matx34f newProjMat = camera.getCameraMatrix() * getTransformFromRT<float>(new_frame.R_, new_frame.t_);

    for (int i = n_old_triang_pts; i < old_frame.keypoints_.size(); i++){
        if (old_frame.matches_[i] != -1){
            cv::Point2f& old_pixel = old_frame.keypoints_[i];
            cv::Point2f& new_pixel = new_frame.keypoints_[i];
            
            // Compute bearing vectors and the angle in between
            cv::Vec3f old_vec = old_frame.R_ * camera.camToWorld(old_pixel);
            cv::Vec3f new_vec = new_frame.R_ * camera.camToWorld(new_pixel);
            float angle = std::acos(old_vec.dot(new_vec) / (cv::norm(old_vec) * cv::norm(new_vec)));

            // Check if the angle between bearing vectors is enough  
            if (angle > bearing_angle_th_){
                // Triangulate
                cv::Mat point_4d;

                cv::triangulatePoints(oldProjMat, 
                                      newProjMat, 
                                      pixToVec<float>(old_pixel), 
                                      pixToVec<float>(new_pixel), 
                                      point_4d);
                
                // Normalize and store the result
                float x = point_4d.at<float>(0,0);
                float y = point_4d.at<float>(0,1);
                float z = point_4d.at<float>(0,2);
                float w = point_4d.at<float>(0,3);

                cv::Point3f pt3d(x/w, y/w, z/w);

                res.emplace_back(pt3d);
            } else {
                old_frame.matches_[i] = -1;
            }
        }
    }
    return res;
}
