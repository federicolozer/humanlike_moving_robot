include <IK_solver.hpp>

int main(int argc, char** argv) {
    std::array<double, 16> O_T_EE_array = { {1, 0, 0, 0.4, 0, 1, 0, 0, 0, 0, 1, 0.4, 0, 0, 0, 1} };
    double q7 = 0;
    std::array<double, 7> q_actual_array = { {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785} };

    std::array< std::array<double, 7>, 4 > q_all = franka_IK_EE(std::array<double, 16> O_T_EE_array,
        double q7,
        std::array<double, 7> q_actual_array)

        std::cout << "Solution = " << q_all << std::endl;
}