#include "eight_point_algorithm/eight_point_algorithm.h"



cv::Matx33d EightPointAlgorithm::computeFoundamentalMatrix(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matched_pts){
    // For some reason SVD is not computed properly, maybe a float precision issue
    int n_matches = matched_pts.size();
    assert(n_matches >= 8 && "More or equal to 8 points are needed to estimate the foundamental matrix");
    
    // Compute the system matrix to estimate the foundamental matrix
    cv::Mat sysmat = getSystemMatrixForFM(matched_pts);

    // Solve
    cv::SVD svd = cv::SVD(sysmat, cv::SVD::FULL_UV);
    cv::Mat F_vec = svd.vt.row(8);

    // Reshape
    cv::Mat F_vec_copy = F_vec.clone();
    cv::Mat F_raw = cv::Mat(cv::Size(3, 3), CV_64FC1, F_vec_copy.data);

    // Force F to have det = 0: this ensures that all the epipolar 
    // lines in an image to intersect at a single point (epipole)
    cv::Mat F = forceMatToZeroDet(F_raw);

    return cv::Matx33d((double*) F.data);
}

cv::Mat EightPointAlgorithm::getSystemMatrixForFM(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matched_pts){
    int n_matches = matched_pts.size();
    cv::Mat sysmat(cv::Size(9, n_matches), CV_64FC1);

    for (int i = 0; i < n_matches; i++){
        const cv::Vec3f& p1 = matched_pts[i].first;
        const cv::Vec3f& p2 = matched_pts[i].second;
        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                sysmat.at<double>(i, j*3 + k) =  p1[j] * p2[k];
            }
            
        }
    }
    return sysmat;
}

cv::Mat EightPointAlgorithm::forceMatToZeroDet(const cv::Mat& matrix){
    assert(matrix.cols == matrix.rows && "The matrix must be squared");
    cv::SVD svd = cv::SVD(matrix);
    cv::Mat u = svd.u;
    cv::Mat vt = svd.vt;
    cv::Mat w = svd.w;

    w.at<double>(matrix.rows-1) = 0.0;
    cv::Mat W = cv::Mat::zeros(cv::Size(3,3), CV_64FC1);
    W.at<double>(0,0) = w.at<double>(0);
    W.at<double>(1,1) = w.at<double>(1);
    W.at<double>(2,2) = w.at<double>(2);
    cv::Mat zero_det_mat = u * W * vt;

    return zero_det_mat; 
}

double EightPointAlgorithm::computeGeometricError(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matches, const cv::Matx33d& F){
    double tot_error = 0;
    int n_matches = matches.size();

    for (const std::pair<cv::Vec3f,cv::Vec3f>& match : matches){
        const cv::Vec3f& p1 = match.first;
        const cv::Vec3f& p2 = match.second;
        cv::Vec3f epi_line_1 = F.t() * p2;
        cv::Vec3f epi_line_2 = F * p1;
        double denom = cv::pow(cv::norm(epi_line_1),2) + cv::pow(cv::norm(epi_line_1),2);
        double error = cv::pow(epi_line_1.dot(p1) + epi_line_2.dot(p2), 2) / denom;
        tot_error += error / n_matches;
    }

    return cv::sqrt(tot_error);
}

double EightPointAlgorithm::computeAlgebraicError(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matches, const cv::Matx33d& F){
    double tot_error = 0;
    int n_matches = matches.size();

    for (const std::pair<cv::Vec3f,cv::Vec3f>& match : matches){
        const cv::Vec3d& p1 = match.first;
        const cv::Vec3d& p2 = match.second;

        double error = cv::pow(cv::norm(p2.t() * F * p1), 2);

        tot_error += error / n_matches;
    }

    return cv::sqrt(tot_error);
}

cv::Matx33d EightPointAlgorithm::computeFoundamentalMatrixNormalized(const std::vector<std::pair<cv::Vec3d,cv::Vec3d>>& matched_pts){
    int matches = matched_pts.size();
    cv::Vec3d mean_pt1 = cv::Vec3d(0,0,0);
    cv::Vec3d mean_pt1 = cv::Vec3d(0,0,0);
}