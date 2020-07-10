// Copyright (c) 2020 by Fujie
//
// file  : main.cpp
// since : 2020-06-11
// desc  : Hand eye calibration main func of mechanical arm(eyeInHand)

#include <algorithm>
#include "utils.h"

int main() {
    std::string rootDir = "../data/20200710/";
    int numPatternViews = 8;

    Eigen::Matrix4f HT2C;
    
    HT2C = calibrate(rootDir, numPatternViews);

    std::cout << "The transform relation between camera and arm base is: " << std::endl;
    std::cout << HT2C << std::endl;

    
    return 0;
}
