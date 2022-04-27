#ifndef ICP_LIDAR
#define ICP_LIDAR

#include <iostream>
#include <knncpp.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#define RAD2DEG(rad) rad*(180/M_PI)
#define DEG2RAD(deg) deg*(M_PI/180)

using namespace std;
using namespace Eigen;

typedef knncpp::Matrixi Matrixi;

class icp_lidar {
    public:
    MatrixXd reference_points, points_to_be_aligned;
    icp_lidar();
    ~icp_lidar() {}
    void knn_kdtree(const MatrixXd& reference_points, const MatrixXd& points_to_be_aligned);
    double* point_based_matching(const MatrixXd& points_pair_a, const MatrixXd& points_pair_b);
    MatrixXd icp(MatrixXd reference_points, MatrixXd points, int max_iterations=100, float distance_threshold=0.3, float convergence_translation_threshold=0.001,
        float convergence_rotation_threshold=0.0001, int point_pairs_threshold=10, bool verbose=true);
    void icp_non_linear();

    private:
    Matrixi indices_;
    MatrixXd distances_;
    Vector2d Split_(string input, char delimiter);
    void push_back_(MatrixXd& m, Vector2d&& values, std::size_t row);
};

#endif