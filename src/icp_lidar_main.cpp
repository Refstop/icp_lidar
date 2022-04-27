#include "icp_lidar/icp_lidar.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::vector<double> eigenVec2stdVec(const VectorXd& evec) {
    std::vector<double> svec;
    for(int i = 0; i < evec.size(); i++) {
        svec.push_back(evec[i]);
    }
    return svec;
}

int main(int argc, char* argv[]) {
    icp_lidar icp;
    MatrixXd result_points = icp.icp(icp.reference_points, icp.points_to_be_aligned, 100, 10);
    // cout << "icp.reference_points:" << endl << icp.reference_points << endl << endl;
    // cout << "icp.points_to_be_aligned:" << endl << icp.points_to_be_aligned << endl << endl;
    // cout << "result_points:" << endl << result_points << endl << endl;
    // const int num = icp.reference_points.cols();
    
    std::vector<double> ref_x = eigenVec2stdVec(icp.reference_points.block<1,30>(0,0));
    std::vector<double> ref_y = eigenVec2stdVec(icp.reference_points.block<1,30>(1,0));
    
    std::vector<double> input_x = eigenVec2stdVec(icp.points_to_be_aligned.block<1,30>(0,0));
    std::vector<double> input_y = eigenVec2stdVec(icp.points_to_be_aligned.block<1,30>(1,0));

    std::vector<double> result_x = eigenVec2stdVec(result_points.block<1,30>(0,0));
    std::vector<double> result_y = eigenVec2stdVec(result_points.block<1,30>(1,0));
    plt::scatter(ref_x, ref_y, 'r');
    plt::plot(input_x, input_y, "b*");
    plt::plot(result_x, result_y, "r*");
    plt::show();
    return 0;
}