/*!
* @file utils.h
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

#include <vector>
#include <string>
#include <assert.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
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
 * @param numSphere    一帧场景中点分割得到点云块的数量 \n
 *
 * @return sphereParas   拟合得到的点云块对应球参数 [x,y,x,r]
 */
std::vector<std::vector<float> > getSpherePara(std::string spherePath, int numSphere);

/** 
 * @brief 利用Tsai-Lenz算法求解eye-in-hand标定问题
 * @param Hcg    相机到机械臂的齐次变换 4*4 \n
 * @param Hgij   相邻两次机械臂的坐标变换 n*4*4 \n
 * @param Hcij   相邻两次相机的坐标变换 n*4*4
 *
 * @return void
 */
void Tsai_lenz(
    Eigen::Matrix4f& Hcg, 
    const std::vector<Eigen::Matrix4f>& Hgij, 
    const std::vector<Eigen::Matrix4f>& Hcij
);

/** 
 * @brief 求解向量对应的反对陈矩阵
 * @param vector3d    三维列向量 3*1 \n
 *
 * @return ret    vector3d对应的反对称矩阵
 */
Eigen::Matrix3f skewMatrix(
    const Eigen::Vector3f& vector3f
);

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
void loadTxt(
    const std::string resPath,
    std::vector< std::vector<float> >& data
);

void run(
    std::string rootDir,
    int numPatterns_a,
    int numPatterns_u
);

#endif