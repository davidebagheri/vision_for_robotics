#include "eight_point_algorithm/eight_point_algorithm.h"



cv::Matx33d EightPointAlgorithm::computeFoundamentalMatrix(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2){
    assert(points_1.size() == points_2.size());
    int n_matches = points_1.size();
    assert(n_matches >= 8 && "More or equal to 8 points are needed to estimate the foundamental matrix");
    
    // Compute the system matrix to estimate the foundamental matrix
    cv::Mat sysmat = getSystemMatrixForFM(points_1, points_2);

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

cv::Mat EightPointAlgorithm::getSystemMatrixForFM(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2){
    int n_matches = points_1.size();
    cv::Mat sysmat(cv::Size(9, n_matches), CV_64FC1);

    for (int i = 0; i < n_matches; i++){
        const cv::Vec3f& p1 = points_1[i];
        const cv::Vec3f& p2 = points_2[i];
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

double EightPointAlgorithm::computeGeometricError(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2, const cv::Matx33d& F){
    assert(points_1.size() == points_2.size());
    double tot_error = 0;
    int n_matches = points_1.size();

    for (int i = 0; i < n_matches; i++){
        const cv::Vec3f& p1 = points_1[i];
        const cv::Vec3f& p2 = points_2[i];
        cv::Vec3f epi_line_1 = F.t() * p2;
        cv::Vec3f epi_line_2 = F * p1;
        double denom = cv::pow(cv::norm(epi_line_1),2) + cv::pow(cv::norm(epi_line_1),2);
        double error = cv::pow(epi_line_1.dot(p1) + epi_line_2.dot(p2), 2) / denom;
        tot_error += error / n_matches;
    }

    return cv::sqrt(tot_error);
}

double EightPointAlgorithm::computeAlgebraicError(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2, const cv::Matx33d& F){
    assert(points_1.size() == points_2.size());
    double tot_error = 0;
    int n_matches = points_1.size();

    for (int i = 0; i < n_matches; i++){
        const cv::Vec3d& p1 = points_1[i];
        const cv::Vec3d& p2 = points_2[i];

        double error = cv::pow(cv::norm(p2.t() * F * p1), 2);

        tot_error += error / n_matches;
    }

    return cv::sqrt(tot_error);
}

cv::Matx33d EightPointAlgorithm::computeFoundamentalMatrixNormalized(const std::vector<cv::Vec3d>& points_1, const std::vector<cv::Vec3d>& points_2){
    assert(points_1.size() == points_2.size());
    int n_matches = points_1.size();

    // Compute normalization matrices
    cv::Matx33d norm_mat_1 = computeNormMatrix(points_1);
    cv::Matx33d norm_mat_2 = computeNormMatrix(points_2);

    // Normalize the points
    std::vector<cv::Vec3d> norm_points_1, norm_points_2; 
    norm_points_1.reserve(n_matches);
    norm_points_2.reserve(n_matches);
    for (int i = 0; i < n_matches; i++){
        norm_points_1.emplace_back(norm_mat_1 * points_1[i]);
        norm_points_2.emplace_back(norm_mat_2 * points_2[i]);
    }

    cv::Matx33d F = computeFoundamentalMatrix(norm_points_1, norm_points_2);

    return norm_mat_2.t() * F * norm_mat_1;
}


cv::Matx33d EightPointAlgorithm::computeNormMatrix(const std::vector<cv::Vec3d>& points){
    int n_matches = points.size();
    cv::Vec3d mean_pt = cv::Vec3d(0,0,0);
    double std_dev = 0;

    // Compute means
    for (int i = 0; i < n_matches; i++){
        mean_pt += points[i] / n_matches;
    }

    // Compute std devs
    for (int i = 0; i < n_matches; i++){
        std_dev += cv::pow(cv::norm((points[i] - mean_pt)), 2) / n_matches;
    }

    std::cout << mean_pt << std::endl;
    std::cout << std_dev << std::endl;

    double s = cv::sqrt(2) / std_dev;

    return cv::Matx33d(s, 0, -s*mean_pt[0], 0, s, -s*mean_pt[1], 0, 0, 1);

}

cv::Matx33d EightPointAlgorithm::estimateEssentialMatrix(const std::vector<cv::Point2f>& points_1, const std::vector<cv::Point2f>& points_2, const cv::Matx33d& K1, const cv::Matx33d& K2){
    // Transform to homogeneous coordinates 
    int n_points = points_1.size();
    std::vector<cv::Vec3d> h_points_1, h_points_2;
    h_points_1.reserve(n_points);
    h_points_2.reserve(n_points);
    for (int i = 0; i < n_points; i++){
        const cv::Point2f& point_1 = points_1[i];
        const cv::Point2f& point_2 = points_2[i];
        h_points_1.emplace_back(cv::Vec3d(point_1.x, point_1.y));
        h_points_2.emplace_back(cv::Vec3d(point_2.x, point_2.y));
    }
    // Foundamental matrix
    cv::Matx33d F =  computeFoundamentalMatrix(h_points_1, h_points_2);
    // Essential matrix 
    cv::Matx33d E = K2.inv().t() * F * K1;  

    return E;
}

std::vector<cv::Matx34d> decomposeEssentialMatrix(const cv::Matx33d& E){
    cv::SVD svd = cv::SVD(E, cv::SVD::FULL_UV);
    cv::Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
    cv::Matx33d U = svd.u;
    cv::Matx33d Vt = svd.vt;

    cv::Matx33d rotation_1 = U * W * Vt;
    cv::Matx33d rotation_2 = U * W.t() * Vt;

    // Force the matrix to have determinant equal to 1
    if (cv::determinant(rotation_1) == -1 ) rotation_1 = -rotation_1;
    if (cv::determinant(rotation_2) == -1 ) rotation_2 = -rotation_2;
    std::vector<cv::Matx33d*> rotations = {&rotation_1, &rotation_2};

    // Possible traslations
    cv::Matx31d t1 = U.col(2);
    cv::Matx31d t2 = -t1;
    std::vector<cv::Matx31d*> traslations = {&t1, &t2};

    // Find all the rotation-traslation combinations
    std::vector<cv::Matx34d> possible_transforms;
    for (cv::Matx33d* rotation : rotations){
        cv::Matx34d possible_transform;
        // Copy the rotation
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                cv::Matx33d& ref_rotation = *rotation;
                possible_transform(i,j) = ref_rotation(i,j);
            }
        }
        for (cv::Matx31d* traslation : traslations){
            // Copy the traslation
            for (int i = 0; i < 3; i++){
                cv::Matx31d& traslation_ref = *traslation;
                possible_transform(i,3) = traslation_ref(i,0);
                possible_transforms.emplace_back(possible_transform);
            }
        }
    }
    return possible_transforms;
}

cv::Matx34d EightPointAlgorithm::disambiguateRelativePose(
                                const std::vector<cv::Matx34d>& possible_T_12,
                                const cv::Matx33d& K1,
                                const cv::Matx33d& K2,
                                const std::vector<cv::Point2f>& points_1,
                                const std::vector<cv::Point2f>& points_2){
    assert(points_1.size() == points_2.size());
    std::vector<int> pos_depth_points(possible_T_12.size(), 0);

    for (int i = 0; i < possible_T_12.size(); i++){
        const cv::Matx34d& T_12 = possible_T_12[i];

        // Compute the two possible projection matrices
        cv::Matx34d M2 = K2 * T_12;
        cv::Matx34d M1(K1(0,0), K1(0,1), K1(0,2), 0,
                       K1(1,0), K1(1,1), K1(1,2), 0,
                       K1(2,0), K1(2,1), K1(2,2), 0);

        // Triangulate points and count those with positive depth
        for (int pt_n = 0; pt_n < points_1.size(); pt_n++){
            cv::Point3f triang_point = linear_triangulator_.triangulatePoints(
                points_1[pt_n], points_2[pt_n], M1, M2);
            if (triang_point.z > 0){
                pos_depth_points[i]++;
            }
        }
    }

    int max_elem = std::max_element(pos_depth_points.begin(), pos_depth_points.end()) 
                    - pos_depth_points.begin();

    return possible_T_12[max_elem];
}