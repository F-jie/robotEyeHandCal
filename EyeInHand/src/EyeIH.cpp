/*!
* @file EyeIH.cpp
* @brief 
* 
* detail description
* 
* @copyright: Copyright 2020 HNU
* @license: GPL
* @birth: created by Fujie on 2020-07-17
* @author Fujie
* @version: V0.0.1
* @revision: last revised by Fujie on 2020-07-17
*/

#include "EyeIH.h"

EyeIH::EyeIH() {
    numPatterns_a = 0;
    numPatterns_u = 0;
    gap = 0;
    rootDir = "";
}

EyeIH::~EyeIH() {}

void EyeIH::set(
        const std::string& armDataPath,
        const int& numPatterns_a, 
        const int& numPatterns_u,
        std::vector< std::vector<float> > centers
) {
    this->numPatterns_a = numPatterns_a;
    this->numPatterns_u = numPatterns_u;
    gap = numPatterns_u / 2;
    this->rootDir = armDataPath;
    for(auto x : centers) {
        this->centers.push_back(x);
    }
}

void EyeIH::loadCamData() {
    
    for(int i = 0; i < numPatterns_a; ++i) {
        Eigen::Matrix4f HC;
        HC << centers[i*4+0][0], centers[i*4+1][0], centers[i*4+2][0], centers[i*4+3][0],
              centers[i*4+0][1], centers[i*4+1][1], centers[i*4+2][1], centers[i*4+3][1],
              centers[i*4+0][2], centers[i*4+1][2], centers[i*4+2][2], centers[i*4+3][2],
              1,                 1,                 1,                 1;
        
        HCs.push_back(HC);
    }
    for(int i = 0; i < numPatterns_u; ++i) {
        int j = (i + gap) % numPatterns_u;
        Hcij.push_back(HCs[i].inverse() * HCs[j]);
    }
}

void EyeIH::loadArmData(std::string motionFile) {

    std::vector< std::vector<float> > armMotion;
    loadTxt(motionFile, armMotion);

    for(auto x : armMotion) {
        Eigen::Matrix4f HG;

        Eigen::Vector3f trans;
        trans << x[0], x[1], x[2];

        Eigen::Quaternionf quat = Eigen::Quaternionf(x[3], x[4], x[5], x[6]);
        Eigen::Matrix3f rot;
        rot = quat.matrix();
        
        HG.block<3,3>(0,0) = rot;
        HG.block<3,1>(0,3) = trans;
        HG.block<1,4>(3,0) = Eigen::Vector4f(0,0,0,1);
        HGs.push_back(HG);
    }
    for(int i = 0; i < numPatterns_u; ++i) {
        int j = (i + gap) % numPatterns_u;
        Hgij.push_back(HGs[i].inverse() * HGs[j]);
    }
}

Eigen::Matrix4f EyeIH::cal() {
    loadCamData();
    std::string armMotionFile = rootDir + "arm/armLoc.txt";
    loadArmData(armMotionFile);
    Tsai_lenz(Hcg, Hgij, Hcij);
    return Hcg;
}

std::vector<Eigen::Matrix4f> EyeIH::get(int idx) {
    std::vector<Eigen::Matrix4f> ret;
    ret.push_back(HCs[idx]);
    ret.push_back(HGs[idx]);
    return ret;
}
