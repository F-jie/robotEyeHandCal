/*!
* @file EyeIH.h
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

#ifndef EYEIH_H_
#define EYEIH_H_

#include "utils.h"

class EyeIH
{
public:
    EyeIH();
    ~EyeIH();

    void set(
        const std::string& armDataPath, 
        const int& numPatterns_a,
        const int& numPatterns_u,
        std::vector< std::vector<float> > centers
    );

    void loadCamData();
    void loadArmData(std::string motionFile);

    Eigen::Matrix4f cal();
    std::vector<Eigen::Matrix4f> get(int idx);

private:
    int numPatterns_a;
    int numPatterns_u;
    int gap;
    std::string rootDir;

    std::vector< std::vector<float> > centers;

    std::vector<Eigen::Matrix4f> HCs;
    std::vector<Eigen::Matrix4f> HGs;

    std::vector<Eigen::Matrix4f> Hcij;
    std::vector<Eigen::Matrix4f> Hgij;
    Eigen::Matrix4f Hcg;

};

#endif