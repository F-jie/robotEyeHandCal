/*!
* @file utils.cpp
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

#include "utils.h"

int sphereExt(std::string cloudPath, std::string spherePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(cloudPath, *cloud) == -1) {
        PCL_ERROR("Point cloud reading failed!");
        return 0;
    }

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
    // viewer->addPointCloud(cloud);
    // viewer->addCoordinateSystem();
    // while(!viewer->wasStopped()) {
    //     viewer->spinOnce (1000);
    //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }

    std::vector<pcl::PointIndices> ece_inlier;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setInputCloud(cloud);
	ece.setClusterTolerance(0.6);
	ece.setMinClusterSize(6000);
	ece.setMaxClusterSize(40000);
	ece.setSearchMethod(tree);
	ece.extract(ece_inlier);

    int counter = 1;
    for(std::vector<pcl::PointIndices>::const_iterator it = ece_inlier.begin(); it != ece_inlier.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
        // viewer->addPointCloud(cloud_cluster);
        // viewer->addCoordinateSystem();
        // while(!viewer->wasStopped()) {
        //     viewer->spinOnce (1000);
        //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        // }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        pcl::io::savePCDFileASCII(spherePath + std::to_string(counter) +".pcd", *cloud_cluster);
        counter++;
    }
    return ece_inlier.size();
}

std::vector< std::vector<float> > getSpherePara(std::string spherePath, int numSphere) {
    std::vector< std::vector<float> > sphereParas;

    for (int i = 1; i <= numSphere; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
        std::string fileName = spherePath  + std::to_string(i) + ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloud_) == -1) {
            PCL_ERROR("couldn't read file pcd\n");
            return sphereParas;
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

        std::vector<float> spherePara;
        if ((*inliersIndices).indices.size() == 0)
			std::cout << "Couldn't find any point that fitted the sphere model." << std::endl;
		else {
			
            spherePara.push_back(float(coefficients->values[0]));
            spherePara.push_back(float(coefficients->values[1]));
            spherePara.push_back(float(coefficients->values[2]));
            spherePara.push_back(float(coefficients->values[3]));
        }
        // std::cout << spherePara[0] << ", "
        //           << spherePara[1] << ", " 
        //           << spherePara[2] << ", " 
        //           << spherePara[3] << std::endl;

        if(spherePara[3] < 15 && spherePara[3] > 5)
            sphereParas.push_back(spherePara);
    }
    sort(sphereParas.begin(), sphereParas.end(), 
        [](auto x, auto y) {
            return x[3] > y[3]; 
        }
    );
    // for (int i = 0; i < sphereParas.size(); ++i) {
    //     for( int j = i + 1; j < sphereParas.size(); ++j) {
    //         if(sphereParas[i][3] < sphereParas[j][3]) {
    //             std::vector<float> tmp = sphereParas[i];
    //             sphereParas[i] = sphereParas[j];
    //             sphereParas[j] = tmp;
    //         }
    //     }
    // }

    return sphereParas;
}

void Tsai_lenz(
    Eigen::Matrix4f& Hcg, 
    const std::vector<Eigen::Matrix4f>& Hgij, 
    const std::vector<Eigen::Matrix4f>& Hcij
) {

    // 保证相机的位置和机械臂末端的位置数量相同
    assert(Hgij.size() == Hcij.size());
    int nStatus = Hgij.size();

    // 保存tmpLeft和tmpRight,使用最小二乘求解P'cg
    Eigen::Matrix<float, Eigen::Dynamic, 3> tmpL;
    Eigen::VectorXf tmpR;
    tmpL.resize(nStatus*3, 3);
    tmpR.resize(nStatus*3, 1);

    for(int i=0;i<nStatus;i++) {
        // 获取对应的旋转矩阵
        Eigen::Matrix3f Rgij = Hgij[i].block<3,3>(0,0);
        Eigen::Matrix3f Rcij = Hcij[i].block<3,3>(0,0);

        // 计算的旋转向量
        Eigen::AngleAxisf rgij;
        Eigen::AngleAxisf rcij;
        rgij.fromRotationMatrix(Rgij);
        rcij.fromRotationMatrix(Rcij);

        // 旋转向量归一化
        float theta_rgij = rgij.angle();
        float theta_rcij = rcij.angle();

        // 基于修正罗德里格斯公式表示的旋转向量
        Eigen::Vector3f Pgij = 2*sin(theta_rgij/2.0)*rgij.axis();
        Eigen::Vector3f Pcij = 2*sin(theta_rcij/2.0)*rcij.axis();

        // 公式：skrw(Pgij+pcij)P'cg=Pcij-Pgij
        Eigen::Matrix3f tmpLeft = skewMatrix(Pgij+Pcij);
        Eigen::Vector3f tmpRight = Pgij - Pcij;

        // 保存tmpLeft和tmpRight，使用最小二乘求解P'cg
        tmpL.block<3,3>(i*3, 0) = tmpLeft;
        tmpR.block<3,1>(i*3, 0) = tmpRight;
    }

    // 最小二乘求解P‘cg
    Eigen::Vector3f Pcg_prime = tmpL.fullPivLu().solve(tmpR);

    // 计算Pcg：Pcg=2*P'cg/sqrt(1+(P'cg)^T*P'cg)
    Eigen::Vector3f Pcg = 2*Pcg_prime/sqrt(1+Pcg_prime.transpose()*Pcg_prime);

    //计算Rcg
    Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
    float Pcg_norm2 = Pcg.transpose()*Pcg;
    Eigen::Matrix3f Rcg = (1-Pcg_norm2/2)*identity+0.5*(Pcg*Pcg.transpose()+sqrt(4-Pcg_norm2)*skewMatrix(Pcg));

    // 保存tmpLeft和tmpRight,使用最小二乘求解Tcg
    Eigen::Matrix<float, Eigen::Dynamic, 3> tmpLL;
    Eigen::VectorXf tmpRR;
    tmpLL.resize(nStatus*3, 3);
    tmpRR.resize(nStatus*3, 1);
    for(int j=0;j<nStatus;j++) {
        // 获取对应的旋转矩阵,平移向量
        Eigen::Matrix3f Rgij = Hgij[j].block<3,3>(0,0);
        Eigen::Matrix3f Rcij = Hcij[j].block<3,3>(0,0);
        Eigen::Vector3f Tgij = Hgij[j].block<3,1>(0,3);
        Eigen::Vector3f Tcij = Hcij[j].block<3,1>(0,3);

        // 计算平移向量Tcg: (Rgij-I)Tcg=Rcg*Tcij-Tgij
        Eigen::Matrix3f tmpAA = Rcij-identity;
        Eigen::Vector3f tmpbb = Rcg*Tgij-Tcij;      

        // 保存tmpLeft和tmpRight，使用最小二乘求解P'cg
        tmpLL.block<3,3>(j*3, 0) = tmpAA;
        tmpRR.block<3,1>(j*3, 0) = tmpbb;  
    }
    
    // 最小二乘求解Tcg
    Eigen::Vector3f Tcg = tmpLL.fullPivLu().solve(tmpRR);

    // 保存Rcg，Tcg到Hcg
    Hcg.block<3,3>(0,0) = Rcg;
    Hcg.block<3,1>(0,3) = Tcg;
    Eigen::Vector4f tmp;
    tmp << 0,0,0,1;
    Hcg.block<1,4>(3,0) = tmp;

    return;
}

Eigen::Matrix3f skewMatrix(const Eigen::Vector3f& vector3f) {
    Eigen::Matrix3f ret;
    ret << 0, -vector3f(2), vector3f(1),
           vector3f(2), 0, -vector3f(0),
           -vector3f(1), vector3f(0), 0;
    return ret;
}

void loadTxt(
    const std::string resPath, 
    std::vector< std::vector<float> >& data
) {
    std::ifstream fp(resPath);
    std::string line;
    
    while(getline(fp, line)) {
        std::vector<float> tmp;
        std::vector<std::string> lineItems;
        lineItems = split(line, " ");
        for(int i=0;i<lineItems.size();++i) {
            tmp.push_back(std::stof(lineItems[i]));
        }
        data.push_back(tmp);
    }
}

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

void run(
    std::string rootDir,
    int numPatterns_a,
    int numPatterns_u
) {
    // std::vector<int> sphereNumPerFrame = {4,7,6,6,6,6,5};
    std::cout << "point cloud clusttering······" << std::endl;
    std::vector<int> sphereNumPerFrame;
    for(int i = 0; i < numPatterns_a; ++i) {
        std::string cloudPath = rootDir+"pcd/"+std::to_string(i+1)+".pcd";
        std::string spherePath = rootDir + "sphere/" + std::to_string(i+1);
        sphereNumPerFrame.push_back(sphereExt(cloudPath, spherePath));
    }

    std::cout << "DONE!\n" << std::endl;
    std::cout << "sphere parameter extracting······" << std::endl;
    std::vector< std::vector<float> > centers;
    int idx = 0;
    for(auto x : sphereNumPerFrame) {
        std::vector< std::vector<float> > sphereParas;
        std::string spherePath = rootDir + "sphere/" + std::to_string(++idx);
        
        sphereParas = getSpherePara(spherePath, x);
        for(auto center : sphereParas) {
            centers.push_back(center);
        }
    }
    std::cout << "DONE!\n" << std::endl;

    std::cout << "**********************************************" << std::endl;
    idx = 0;
    std::cout << "Sphere parameter extracted" << std::endl;
    for(auto x : centers) {
        if(idx % 4 == 0) {
            std::cout << idx / 4 + 1 <<"th farme\t" << std::endl;
        }
        idx++;
        std::cout << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << std::endl;
    }
    std::cout << "**********************************************\n" << std::endl;

    std::cout << "Start calbrition!!" << std::endl;
    EyeIH calibrator = EyeIH();
    Eigen::Matrix4f HC2G;
    calibrator.set(rootDir, numPatterns_a, numPatterns_u, centers);
    HC2G = calibrator.cal();

    std::cout << "**********************************************" << std::endl;
    std::cout << "Result: " << std::endl;
    std::cout << HC2G << std::endl;
    std::cout << "**********************************************\n" << std::endl;

    std::cout << "**********************************************" << std::endl;
    std::cout << "Accuracy: " << std::endl;
    std::vector<Eigen::Matrix4f> ans = calibrator.get(5);
    std::cout << ans[1] * HC2G.inverse() * ans[0] << std::endl;

    ans = calibrator.get(6);
    std::cout << ans[1] * HC2G.inverse() * ans[0] << std::endl;
    std::cout << "**********************************************" << std::endl;
}
