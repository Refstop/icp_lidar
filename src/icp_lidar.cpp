#include "icp_lidar/icp_lidar.h"

icp_lidar::icp_lidar() {
    this->reference_points.resize(1,2);
    this->points_to_be_aligned.resize(1,2);

    // ifstream reference_points("../example/reference_points.txt");
    ifstream reference_points("../example/true_data.txt");
	if(!reference_points.is_open()) {
        cout << "Error opening file" << endl;
        exit(1);
    }
    int i = 0;
	for(string line; reference_points.peek() != EOF; i++) {
        getline(reference_points, line);
        push_back_(this->reference_points, Split_(line, ' '), i);
    }

    // ifstream points_to_be_aligned("../example/points_to_be_aligned.txt");
    ifstream points_to_be_aligned("../example/moved_data.txt");
	if(!points_to_be_aligned.is_open()) {
        cout << "Error opening file" << endl;
        exit(1);
    }
    
    i = 0;
	for(string line; points_to_be_aligned.peek() != EOF; i++) {
        getline(points_to_be_aligned, line);
        push_back_(this->points_to_be_aligned, Split_(line, ' '), i);
    }
    this->reference_points.transposeInPlace();
    this->points_to_be_aligned.transposeInPlace();
}

void icp_lidar::knn_kdtree(const MatrixXd& reference_points, const MatrixXd& points_to_be_aligned) {
    knncpp::KDTreeMinkowskiX<double, knncpp::EuclideanDistance<double>> kdtree(reference_points);

    kdtree.setBucketSize(1);
    kdtree.build();

    kdtree.query(points_to_be_aligned, 1, indices_, distances_);
    cout << "indices_.cols():" << endl << indices_.cols() << endl << endl;
    cout << "distances_.cols():" << endl << distances_.cols() << endl << endl;
}

double* icp_lidar::point_based_matching(const MatrixXd& points_pair_a, const MatrixXd& points_pair_b) {
    static double xytheta[3];
    double x, y, xp, yp, x_mean = 0, y_mean = 0, xp_mean = 0, yp_mean = 0;
    double s_x_xp = 0, s_y_yp = 0, s_x_yp = 0, s_y_xp = 0;
    int n = points_pair_a.rows();
    if(n == 0) return NULL;
    for(int i = 0; i < n; i++) {
        x = points_pair_a(i,0);
        y = points_pair_a(i,1);
        xp = points_pair_b(i,0);
        yp = points_pair_b(i,1);
        

        x_mean += x;
        y_mean += y;
        xp_mean += xp;
        yp_mean += yp;   
    }

    x_mean /= n;
    y_mean /= n;
    xp_mean /= n;
    yp_mean /= n;
    
    for(int i = 0; i < n; i++) {
        x = points_pair_a(i,0);
        y = points_pair_a(i,1);
        xp = points_pair_b(i,0);
        yp = points_pair_b(i,1);

        s_x_xp += (x - x_mean)*(xp - xp_mean);
        s_y_yp += (y - y_mean)*(yp - yp_mean);
        s_x_yp += (x - x_mean)*(yp - yp_mean);
        s_y_xp += (y - y_mean)*(xp - xp_mean);
    }

    xytheta[2] = atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp);
    xytheta[0] = xp_mean - (x_mean*cos(xytheta[2]) - y_mean*sin(xytheta[2]));
    xytheta[1] = yp_mean - (x_mean*sin(xytheta[2]) + y_mean*cos(xytheta[2]));

    return xytheta;
}

MatrixXd icp_lidar::icp(MatrixXd reference_points, MatrixXd points, int max_iterations, float distance_threshold, float convergence_translation_threshold,
        float convergence_rotation_threshold, int point_pairs_threshold, bool verbose) {
    for(int iter_num = 0; iter_num < max_iterations; iter_num++) {
        if(verbose) cout << "------ iteration " << iter_num << " ------" << endl;
        knn_kdtree(reference_points, points);

        // cout << "reference_points:" << endl << reference_points << endl << endl;
        // cout << "points:" << endl << points << endl << endl;
        MatrixXd points_pair_a(1,2), points_pair_b(1,2);
        int nn_index = 0;
        for(int i = 0; i < distances_.size(); i++) {
            if(distances_(nn_index) < distance_threshold) {
                push_back_(points_pair_a, points.block<2,1>(0,nn_index), nn_index);
                push_back_(points_pair_b, reference_points.block<2,1>(0,indices_(nn_index)), nn_index);
                nn_index++;
            }
        }
        // cout << "points_pair_a:" << endl << points_pair_a << endl << endl;
        // cout << "points_pair_b:" << endl << points_pair_b << endl << endl;
        if(verbose) cout << "number of pairs found:" << points_pair_a.rows() << endl;
        if(points_pair_a.rows() < point_pairs_threshold) {
            if(verbose) cout << "No better solution can be found (very few point pairs)!" << endl;
            break;
        }
        double *xytheta = point_based_matching(points_pair_a, points_pair_b);
        // for(int i = 0; i < 3; i++) {
        //     cout << "xytheta[" << i << "]:" << endl << xytheta[i] << endl << endl;
        // }
        
        if(xytheta != NULL) {
            if(verbose) {
                cout << "Rotation: " << RAD2DEG(xytheta[2]) << "degrees" << endl;
                cout << "Translation: " << xytheta[0] << ", " << xytheta[1] << endl;
            }
        }
        else {
            if(verbose) cout << "No better solution can be found!" << endl;
            break;
        }
        Matrix2d rot;
        rot << cos(xytheta[2]), -sin(xytheta[2]),
                sin(xytheta[2]), cos(xytheta[2]);
        MatrixXd aligned_points = rot * points;
        for(int j = 0; j < aligned_points.cols(); j++) {
            aligned_points.block<2,1>(0,j) += Vector2d(xytheta[0],xytheta[1]);
        }

        points = aligned_points;
        
        if( (abs(xytheta[2]) < convergence_rotation_threshold)
            && (abs(xytheta[0]) < convergence_translation_threshold)
            && (abs(xytheta[1]) < convergence_translation_threshold) ) {
            if(verbose) cout << "Converged!" << endl;
            break;
        }
    }
    return points;
}

Vector2d icp_lidar::Split_(string input, char delimiter) {
    Vector2d answer;
    stringstream ss(input);
    string temp;

    for(int i = 0; getline(ss, temp, delimiter); i++) {
        answer(i) = stod(temp);
    }
    return answer;
}

void icp_lidar::push_back_(MatrixXd& m, Vector2d&& values, std::size_t row) {
    if(row >= m.rows()) {
        m.conservativeResize(row + 1, Eigen::NoChange);
    }
    m.row(row) = values;
}