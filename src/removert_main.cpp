#include "removert/Removerter.h"
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

void PCATest() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::string fileName = "/home/zhuhongwei/bunny/reconstruction/bun_zipper_res2.ply";
    pcl::io::loadPLYFile(fileName.c_str(), *cloud);

    Eigen::Vector4f pcaCentroid; // 点云的质心
    pcl::compute3DCentroid(*cloud, pcaCentroid); // 计算质心
    Eigen::Matrix3f covariance; // 协方差矩阵
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors(); // 特征向量
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues(); // 特征值
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    std::cout << "原始点云的特征值value(3x1):\n" << eigenValuesPCA << std::endl;
    std::cout << "原始点云的特征向量vector(3x3):\n" << eigenVectorsPCA << std::endl;
    std::cout << "原始点云的质心(4x1):\n" << pcaCentroid << std::endl;
/*
// 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCA<pcl::PointXYZ> pca;
pca.setInputCloud(cloudSegmented);
pca.project(*cloudSegmented, *cloudPCAprojection);
std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
*/
    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
    tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose(); // R
    tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>());//  -R*t
    tm_inv = tm.inverse();

    std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
    std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformedCloud, tm); // 原始点云变换到特征点云

    pcl::PointXYZ min_p1, max_p1;
    Eigen::Vector3f c_feature, c;
    pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
    c_feature = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

    std::cout << "特征形心(3x1):\n" << c_feature << std::endl;

    Eigen::Affine3f tm_inv_aff(tm_inv);
    pcl::transformPoint(c_feature, c, tm_inv_aff); // 特征形心去求原始形心

    std::cout << "原始形心(3x1):\n" << c << std::endl;

// ！最重要的变量!
// 代表：任意形状点云，它的三个特征主方向的长度值
    Eigen::Vector3f feature;
    feature = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    float vectorXYZ_mean = (feature(0) + feature(1) + feature(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

    std::cout << "width = " << feature(0) << std::endl; // 点云的宽（特征方向，非原始xyz方向）
    std::cout << "height = " << feature(1) << std::endl; // 点云的高（特征方向，非原始xyz方向）
    std::cout << "depth = " << feature(2) << std::endl; // 点云的长（特征方向，非原始xyz方向）
    std::cout << "mean = " << vectorXYZ_mean << std::endl; // 点云长宽高的平均值

    const Eigen::Quaternionf box_rotate_feature(Eigen::Quaternionf::Identity());
    const Eigen::Vector3f box_pose_feature(c_feature); // 点云的特征形心

    const Eigen::Quaternionf box_rotate_origin(tm_inv.block<3, 3>(0, 0));
    const Eigen::Vector3f box_pose_origin(c); // 原始点云的形心

    pcl::PointXYZ center; // 原始点云的质心
    center.x = pcaCentroid(0); // 原始点云质心的xyz坐标
    center.y = pcaCentroid(1);
    center.z = pcaCentroid(2);
// 原始点云的三个特征方向的轴射线，每个轴射线设置一个固定长度，每个轴的方向取决于点云的三个特征主方向
// 每个轴射线的末端点 = 轴长 * 轴方向 + 质心坐标
    pcl::PointXYZ x_axis_origin;
    x_axis_origin.x = vectorXYZ_mean * eigenVectorsPCA(0, 0) + center.x;
    x_axis_origin.y = vectorXYZ_mean * eigenVectorsPCA(1, 0) + center.y;
    x_axis_origin.z = vectorXYZ_mean * eigenVectorsPCA(2, 0) + center.z;
    pcl::PointXYZ y_axis_origin;
    y_axis_origin.x = vectorXYZ_mean * eigenVectorsPCA(0, 1) + center.x;
    y_axis_origin.y = vectorXYZ_mean * eigenVectorsPCA(1, 1) + center.y;
    y_axis_origin.z = vectorXYZ_mean * eigenVectorsPCA(2, 1) + center.z;
    pcl::PointXYZ z_axis_origin;
    z_axis_origin.x = vectorXYZ_mean * eigenVectorsPCA(0, 2) + center.x;
    z_axis_origin.y = vectorXYZ_mean * eigenVectorsPCA(1, 2) + center.y;
    z_axis_origin.z = vectorXYZ_mean * eigenVectorsPCA(2, 2) + center.z;


    pcl::PointXYZ center_zero; // 原点
    center_zero.x = 0.0;
    center_zero.y = 0.0;
    center_zero.z = 0.0;
    Eigen::Affine3f tm_aff(tm);
    Eigen::Vector3f px = eigenVectorsPCA.col(0); // 原点云的三个特征向量
    Eigen::Vector3f py = eigenVectorsPCA.col(1);
    Eigen::Vector3f pz = eigenVectorsPCA.col(2);
    pcl::transformVector(px, px, tm_aff); // 计算得到在特征空间下，xyz三个方向的值
    pcl::transformVector(py, py, tm_aff);
    pcl::transformVector(pz, pz, tm_aff);
// 原始点云的三个特征方向，均变换到原始坐标系xyz方向
    pcl::PointXYZ x_axis_feature;
    x_axis_feature.x = vectorXYZ_mean * px(0);
    x_axis_feature.y = vectorXYZ_mean * px(1);
    x_axis_feature.z = vectorXYZ_mean * px(2);
    pcl::PointXYZ y_axis_feature;
    y_axis_feature.x = vectorXYZ_mean * py(0);
    y_axis_feature.y = vectorXYZ_mean * py(1);
    y_axis_feature.z = vectorXYZ_mean * py(2);
    pcl::PointXYZ z_axis_feature;
    z_axis_feature.x = vectorXYZ_mean * pz(0);
    z_axis_feature.y = vectorXYZ_mean * pz(1);
    z_axis_feature.z = vectorXYZ_mean * pz(2);


//visualization
    pcl::visualization::PCLVisualizer viewer;

//// 特征点云（绿色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(transformedCloud, 0, 255, 0); // rgb
    viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
    viewer.addCube(box_pose_feature, box_rotate_feature, feature(0), feature(1), feature(2), "box_feature");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                       pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                       "box_feature");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "box_feature"); // rgb

// 特征点云的三个轴
    viewer.addArrow(x_axis_feature, center_zero, 1.0, 0.0, 0.0, false, "arrow_X"); // 红， 格式rgb
    viewer.addArrow(y_axis_feature, center_zero, 0.0, 1.0, 0.0, false, "arrow_Y"); // 绿， 格式rgb
    viewer.addArrow(z_axis_feature, center_zero, 0.0, 0.0, 1.0, false, "arrow_Z"); // 蓝， 格式rgb

// 原始点云（红色）
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 0, 0); // rgb
//    viewer.addPointCloud(cloud, color_handler, "cloud");
//    viewer.addCube(box_pose_origin, box_rotate_origin, feature(0), feature(1), feature(2), "box_origin");
//    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
//                                       pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
//                                       "box_origin");
//    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "box_origin");
//
//// 原始点云的三个轴
//    viewer.addArrow(x_axis_origin, center, 1.0, 0.0, 0.0, false, "arrow_x");
//    viewer.addArrow(y_axis_origin, center, 0.0, 1.0, 0.0, false, "arrow_y");
//    viewer.addArrow(z_axis_origin, center, 0.0, 0.0, 1.0, false, "arrow_z");

    viewer.addCoordinateSystem(0.5f * vectorXYZ_mean);
    viewer.setBackgroundColor(1.0, 1.0, 1.0);
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "removert");
    ROS_INFO("\033[1;32m----> Removert Main Started.\033[0m");

/*
 *  PCA test for Stanford Rabbit Model, if you want to see the PCA test ,you can use   PCATest() function.
 *  May you need to download the The Stanford 3D Scanning Repository(http://graphics.stanford.edu/data/3Dscanrep/)
 */
//    PCATest();

    Removerter RMV;
    RMV.run();

    ros::spin();
    return 0;
}