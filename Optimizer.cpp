#pragma once

#include <Eigen/Dense>
#include "IK_solver.hpp"
#include <iostream>


int main(int argc, char** argv) {
    std::array<double, 16> O_T_EE_array = { {1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.2, 0.0, 0.4, 1.0} };
    Eigen::Map< Eigen::Matrix<double, 4, 4> > O_T_EE(O_T_EE_array.data());
    double q7 = -0.285;
    std::array<double, 7> q_actual_array = { {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785} };

    std::cout << O_T_EE << std::endl;


    // first algorithm
    std::array< std::array<double, 7>, 4 > q_all = franka_IK_EE(O_T_EE, q7, q_actual_array);

    int cnt = 1;
    for (std::array<double, 7> rows : q_all) {
        std::cout << "Solution " << cnt++ << " = [";
        for (double i : rows) {
            std::cout << i << " ";
        }
        std::cout << "]" << std::endl;
    }


    std::cout << std::endl << "------------------" << std::endl << std::endl;


    std::array<double, 7> q = franka_IK_EE_CC(O_T_EE, q7, q_actual_array);

    std::cout << std::endl << "Solution = [";
    for (double i : q) {
        std::cout << i << " ";
    }
    std::cout << "]" << std::endl;

}