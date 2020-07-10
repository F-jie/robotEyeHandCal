// Copyright (c) 2020 by Fujie
//
// file  : utils.h
// since : 2020-06-11
// desc  : Hand eye calibration utils of mechanical arm(eyeInHand)

#include<vector>
#include<string>
#include<fstream>
#include<assert.h>
#include<iostream>
#include <algorithm>

#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>  
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


#ifndef UTILS_H_
#define UTILS_H_

/** 
 * @brief 使用分割符/sep/分割字符串/text/
 * @param text        原始字符产 \n
 * @param sep         分隔符 \n
 *
 * @return tockens    保存分割结果的字符串数组
 */
std::vector<std::string> split(
    const std::string &text, 
    char* sep
);

/** 
 * @brief 从文本文件中加载空格分割的数据，保存为浮点数矩阵
 * @param resPath    文本文件路径 \n
 * @param data       文本文件对应数据 \n
 *
 * @return void
 */
void loadData(
    const std::string resPath,
    std::vector< std::vector<float> >& data
);
/** 
 * @brief 使用聚类对点云进行分割，并分别保存分割得到的点云块
 * @param cloudPath     待分割点云文件 \n
 * @param spherePath    分割得到点云保存路径 \n
 *
 * @return int   分割得到的点云块数量
 */
int sphereExt(std::string cloudPath, std::string spherePath);


/** 
 * @brief 使用聚类对点云进行分割，并分别保存分割得到的点云块
 * @param spherePath   待拟合的点云的路径，eg：/path_to_sphere/scene_num \n
 *
 * @return sphereParas   拟合得到的点云块对应球参数 [x,y,x,r]
 */
std::vector<float> getSpherePara(std::string spherePath);

/** 
 * @brief 使用聚类对点云进行分割，并分别保存分割得到的点云块
 * @param rootDir   数据存放路径 \n
 * @param numPatternViews    标定及验证所需点云帧数 \n
 *
 * @return 标定结果
 */
Eigen::Matrix4f calibrate(std::string rootDir, int numPatternViews);
#endif