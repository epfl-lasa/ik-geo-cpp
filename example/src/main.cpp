#include <iostream>
#include "ik_geo.h"
#include <vector>


using namespace ik_geo;

int main(int argc, char const *argv[])
{
    Robot robot = Robot::ur5();
    // Create identity matrix
    double rotation_matrix[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    double position_vector[3] = {0.0, 0.0, 0.0};

    std::vector<ik_geo::Solution> solutions;
    robot.ik(rotation_matrix, position_vector, solutions);

    for (auto &solution : solutions) {
        std::cout << "Solution: ";
        for (size_t i = 0; i < 6; i++) {
            std::cout << solution.q[i] << " ";
        }
        std::cout << "Is LS: " << solution.is_ls << std::endl;
    }
    return 0;
}
