#include "utils.h"

int main() { 
    std::string rootDir = "../data/20200715/";
    
    run(rootDir, 7, 5);
    //
    // std::cout << "Estimation accuracy: " << std::endl;
    // Eigen::Matrix4f HW2G = calibrator.getHW2G(5);
    // Eigen::Matrix4f centers = calibrator.getCentersInLC(5);

    // std::cout << HW2G << std::endl;
    // std::cout << centers << std::endl;
    // std::cout << HW2G * (HC2G.inverse()) * centers << std::endl;

    // Eigen::Matrix4f HW2G1 = calibrator.getHW2G(6);
    // Eigen::Matrix4f centers1 = calibrator.getCentersInLC(6);

    // std::cout << HW2G1 * (HC2G.inverse()) * centers1 << std::endl;

    return 0;
}