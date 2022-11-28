#include "poly_traj_utils_total.hpp"
#include "matplotlib-cpp/matplotlibcpp.h"
#include <iostream>
namespace plt = matplotlibcpp;

using namespace std;
int main()
{
    poly_traj::MinJerkOpt test_minco; //init

    //reset
    Eigen::Matrix<double, 4, 3> head_state = Eigen::Matrix<double,4,3>::Zero();
    Eigen::Matrix<double, 4, 3> tail_state;
    tail_state << 20, 0, 0, 
    0, 0, 0,
    0, 0, 0,
    0, 0, 0;
    int pieceNum = 2;
    test_minco.reset(head_state, tail_state, pieceNum, 4);

    //generate trajectory
    Eigen::MatrixXd inPs;
    Eigen::VectorXd ts;
    inPs.resize(3,pieceNum-1);
    ts.resize(2);
    inPs << 10, 0, 0;
    ts << 1, 2;
    test_minco.generate(inPs, ts);

    double duration =  test_minco.getTraj().getTotalDuration();
    size_t N = 1000;
    double step = duration / N;
    std::vector<double> times, pos_x, pos_y, pos_z;
    for (size_t i = 0; i < N; i++)
    {
        double time = i*step;
        times.push_back(time); //times
        pos_x.push_back(test_minco.getTraj().getPos(time)(0)); //position
        pos_y.push_back(test_minco.getTraj().getPos(time)(1));
        pos_z.push_back(test_minco.getTraj().getPos(time)(2));

    }
    
    plt::figure_size(1200,780);
    plt::subplot(3,3,1); plt::plot(times, pos_x); 
    plt::subplot(3,3,2); plt::plot(times, pos_y);
    plt::subplot(3,3,3); plt::plot(times, pos_z);
    plt::show();
}

