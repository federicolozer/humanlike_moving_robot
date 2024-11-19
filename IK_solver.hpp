#pragma once

#ifdef IK_SOLVER_EXPORTS
#define FRANKA_IK __declspec(dllexport)
#else
#define FRANKA_IK __declspec(dllimport)
#endif


#ifndef IK_solver
#define IK_solver

#define _USE_MATH_DEFINES

#include <array>
#include "Eigen/Dense"



FRANKA_IK void error();

FRANKA_IK void error(int n, double val);

// inverse kinematics w.r.t. End Effector Frame (using Franka Hand data)
FRANKA_IK std::array< std::array<double, 7>, 4 > franka_IK_EE(Eigen::Map< Eigen::Matrix<double, 4, 4> > O_T_EE, double q7, std::array<double, 7> q_actual_array);

// "Case-Consistent" inverse kinematics w.r.t. End Effector Frame (using Franka Hand data)
FRANKA_IK std::array<double, 7> franka_IK_EE_CC(Eigen::Map< Eigen::Matrix<double, 4, 4> > O_T_EE, double q7, std::array<double, 7> q_actual_array);

#endif
