﻿#pragma once

#ifndef UTILS
#define UTILS

#include <eigen3/Eigen/Dense>
#include <array>
#include <cmath>
#include <iostream>

boost::array<double, 7> q_actual_array = {{0, -0.785398163397, 0, -2.3561944899, 0, 1.57079632679, 0.785398163397}};



void printFrame(Eigen::Matrix4d O_T_EE_tmp) {
    std::array<float, 3> euler = frameToEuler(O_T_EE_tmp); 
    std::string msg1 = "rosrun gazebo_ros spawn_model -file /home/lozer/franka_emika_ws/src/humanlike_moving_robot/data/models/frame/model.sdf -sdf -model frame ";
    std::stringstream ss;
    ss << "-x " << O_T_EE_tmp(0, 3) << " -y " << O_T_EE_tmp(1, 3) << " -z " << O_T_EE_tmp(2, 3) << " -R " << euler[0]<<  " -P " << euler[1] << " -Y " << euler[2];
    std::string msg2 = ss.str();
    
    int res = system("rosservice call gazebo/delete_model '{model_name: frame}'");
    res = system((msg1+msg2).c_str());
}



Eigen::Matrix4d baseCoordTransf(Eigen::Matrix4d O_T_EE_mat, int mode) {
    Eigen::Matrix4d O_T_EE_tmp;   

    if (mode == 0) { // vert
        O_T_EE_tmp = Eigen::Matrix4d(O_T_EE_mat.data());
    }
    else if (mode == 1) { // horz
        Eigen::Matrix4d O_T_EE_rot(O_T_EE_mat.data());
        Eigen::Matrix4d baseToWall_rot;
        baseToWall_rot << 0.0, 0.0, 1.0, -0.333,
                            0.0, -1.0, 0.0, 0.0,
                            1.0, 0.0, 0.0, 0.7,
                            0.0, 0.0, 0.0, 1.0;
        O_T_EE_tmp = baseToWall_rot.inverse()*O_T_EE_rot;
    }
    else if (mode == 2) { // ceil
        Eigen::Matrix4d O_T_EE_rot(O_T_EE_mat.data());
        Eigen::Matrix4d baseToWall_rot;
        baseToWall_rot << 1.0, 0.0, 0.0, 0.0,
                            0.0, -1.0, 0.0, 0.0,
                            0.0, 0.0, -1.0, 1.033,
                            0.0, 0.0, 0.0, 1.0;
        O_T_EE_tmp = baseToWall_rot.inverse()*O_T_EE_rot;
    }
    
    return O_T_EE_tmp;
}



bool IK_check(Eigen::Map< Eigen::Matrix4d > O_T_EE, double q7, int mode) {
    Eigen::Matrix4d O_T_EE_tmp = baseCoordTransf(O_T_EE, mode);
    Eigen::Map<Eigen::Matrix4d> O_T_EE_new(O_T_EE_tmp.data());

    boost::array<boost::array<double, 7>, 4> q_array_list = IK_solver(O_T_EE_new, q7, q_actual_array, false);

    bool result = false;
    bool valid;
    for (int i=0; i<4; i++) {
        valid = true;
        for (int j=0; j<7; j++) {
            if (std::isnan(q_array_list[i][j])) {
                valid = false;
            }
        }
        if (valid) {
            result = true;
            break;
        }
    }

    return result;
}

#endif