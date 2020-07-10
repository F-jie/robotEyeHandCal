// Copyright (c) 2020 by Fujie
//
// file  : utils.cpp
// since : 2020-06-11
// desc  : Hand eye calibration utils of mechanical arm(eyeInHand)

#include "utils.h"

std::vector<std::string> split(
    const std::string &text, 
    char* sep
) {
    std::vector<std::string> tokens; 
    std::size_t start = 0, end = 0; 
    while ((end = text.find(*sep, start)) != std::string::npos) { 
        tokens.push_back(text.substr(start, end - start));  
        start = end + 1; 
    } 
    tokens.push_back(text.substr(start)); 
    return tokens; 
}

void loadData(
    const std::string resPath, 
    std::vector< Eigen::Vector4f >& data
) {
    std::ifstream fp(resPath);
    std::string line;
    
    while(getline(fp, line)) {
        Eigen::Vector4f loc;
        std::vector<float> tmp;
        std::vector<std::string> lineItems;
        lineItems = split(line, ",");
        for(int i=0;i<lineItems.size();++i) {
            tmp.push_back(std::stof(lineItems[i]));
        }
        loc << tmp[0], tmp[1], tmp[2], 1.0;
        data.push_back(loc);
    }
}

int sphereExt(std::string cloudPath, std::string spherePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(cloudPath, *cloud) == -1) {
        PCL_ERROR("Point cloud reading failed!");
        return 0;
    }
    pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
    // viewer->addPointCloud(cloud);
    // viewer->addCoordinateSystem();
    // while(!viewer->wasStopped()) {
    //     viewer->spinOnce (1000);
    //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;  //创建滤波器对象
    pass.setInputCloud(cloud);              //设置待滤波的点云
    pass.setFilterFieldName("z");             //设置在Z轴方向上进行滤波
    pass.setFilterLimits(10, maxPt.z);             //设置滤波范围为0~1,在范围之外的点会被剪除
    pass.filter(*cloud_sphere);
    cloud_sphere->width = cloud_sphere->points.size();
    cloud_sphere->height = 1;
    pcl::io::savePCDFileASCII(spherePath + ".pcd", *cloud_sphere);

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
    // viewer->addPointCloud(cloud_sphere);
    // viewer->addCoordinateSystem();
    // while(!viewer->wasStopped()) {
    //     viewer->spinOnce (1000);
    //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }

    return 1;
}

std::vector<float> getSpherePara(std::string spherePath) {
    std::vector<float> spherePara;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    std::string fileName = spherePath + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloud_) == -1) {
        PCL_ERROR("couldn't read file pcd\n");
        return spherePara;
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(cloud_);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setMaxIterations(100000);

    pcl::PointIndices::Ptr inliersIndices(new pcl::PointIndices);
    seg.segment(*inliersIndices, *coefficients);

    if ((*inliersIndices).indices.size() == 0)
        std::cout << "Couldn't find any point that fitted the sphere model." << std::endl;
    else {
        
        spherePara.push_back(float(coefficients->values[0]));
        spherePara.push_back(float(coefficients->values[1]));
        spherePara.push_back(float(coefficients->values[2]));
        spherePara.push_back(float(coefficients->values[3]));
    }

    std::cout << spherePara[0] << ", "
              << spherePara[1] << ", " 
              << spherePara[2] << ", " 
              << spherePara[3] << std::endl;

    return spherePara;
}

Eigen::Matrix4f calibrate(std::string rootDir, int numPatternViews) {
    // vector used to save the sphere parameters <x ,y, z, r> 
    std::vector< Eigen::Vector4f > sphereParas;
    std::vector< Eigen::Vector4f > tcpLoc;

    Eigen::Matrix4f PB;
    Eigen::Matrix4f PC;

    Eigen::Matrix4f HT2C;

    int acc = 0;

    // remove the host table plane with pass through filter 
    for(int i = 0; i < numPatternViews; ++i) {
        std::string cloudPath = rootDir+"pcd/"+std::to_string(i+1)+".pcd";
        std::string spherePath = rootDir + "sphere/" + std::to_string(i+1);
        sphereExt(cloudPath, spherePath);
    }

    loadData(rootDir + "arm/tcpLoc.txt", tcpLoc);

    // use pcl to extract sphere parameters
    std::cout << "Sphere parameters<x, y, z, r>: " << std::endl;
    for(int i=0; i< numPatternViews; ++i) {
        std::vector<float> spherePara;
        std::string spherePath = rootDir + "sphere/" + std::to_string(i+1);
        
        spherePara = getSpherePara(spherePath);

        // std::cout << tcpLoc[i] << std::endl; 
        Eigen::Vector4f paraVec;
        paraVec << spherePara[0], spherePara[1], spherePara[2], 1.0;
        sphereParas.push_back(paraVec);
    }

    // build tcp loc matrix in arm base coord and point cloud coord
    for(int i = 0; i < 4; ++i) {
        PB.block<4, 1>(0, i) = tcpLoc[i];
        PC.block<4, 1>(0, i) = sphereParas[i];
    }

    HT2C = PB * PC.inverse();

    for(int i = 4; i < numPatternViews; ++i) {
        acc = acc + (HT2C * sphereParas[i] - tcpLoc[i]).norm();
    }

    std::cout << "The approximate accuracy is: " << acc / 4.0 << " mm" << std::endl;
    
    return HT2C;
}
