//
// Created by zhuhongwei on 2023/3/2.
//



#include <pcl/point_cloud.h>
#include "removert/RANSAC_Seg.h"

RANSAC_seg::RANSAC_seg(Removerter* RMT){
    removerter = RMT;
};


void RANSAC_seg::RANSAC_segmentation() {

//    pcl::PointCloud<PointType>::Ptr landscape, ground;
//    landscape.reset(new pcl::PointCloud<PointType>());

    pcl::PointCloud<PointType>::Ptr ground;
    ground.reset(new pcl::PointCloud<PointType>());

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    pcl::ExtractIndices<PointType> extract;
    // 初始化RANSAC分割器
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(removerter->seg_threshold_);
    seg.setMaxIterations(removerter->max_iterations_);
    removerter->map_global_curr_->clear();
    ROS_INFO_STREAM("\033[1;32m Start ground segmentation" << "\033[0m");
    for(std::size_t idx_scan=0; idx_scan < removerter->scans_.size(); ++idx_scan) {

        pcl::PointCloud<PointType>::Ptr landscape (new pcl::PointCloud<PointType>);

        pcl::PointCloud<PointType>::Ptr scan = removerter->scans_.at(idx_scan);
        Eigen::Matrix4d scan_pose = removerter->scan_poses_.at(idx_scan);

        seg.setInputCloud(scan);
        seg.segment(*inliers, *coefficients);
        // RANSAC
        extract.setInputCloud(scan);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground);

        extract.setNegative(true);
        extract.filter(*landscape);
        /// current coord landscape points
        obstacle_cloud_vector.push_back(landscape);

        // transform current scan to global coord
        pcl::PointCloud<PointType>::Ptr scan_ground_global_coord(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*ground, *scan_ground_global_coord, removerter->kSE3MatExtrinsicLiDARtoPoseBase); /// 将点云从雷达坐标系变换到位姿统一（kitty就是以相机的坐标为基准）的坐标系下
        pcl::transformPointCloud(*scan_ground_global_coord, *scan_ground_global_coord, scan_pose); // 变换到第一针坐标系下
        /// 先把地面点作为全局的静态点保存下来，保存在全局的地图中
        *(removerter->map_global_static) += *scan_ground_global_coord;
        scan_ground_global_coord->clear();

        pcl::PointCloud<PointType>::Ptr scan_landscape_global_coord(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*landscape, *scan_landscape_global_coord, removerter->kSE3MatExtrinsicLiDARtoPoseBase); /// 将点云从雷达坐标系变换到位姿统一（kitty就是以相机的坐标为基准）的坐标系下
        pcl::transformPointCloud(*scan_landscape_global_coord, *scan_landscape_global_coord, scan_pose); // 变换到第一针坐标系下
//        landscape.reset();
        /// 把去除了地面点的帧拼起来并保存成地图，我们尽量使用removert里面的变量
        *(removerter->map_global_curr_) += *scan_landscape_global_coord;
        scan_landscape_global_coord->clear();
    }
    ROS_INFO_STREAM("\033[1;32m  End ground segmentation" << "\033[0m");
    std::string non_ground_file_name = removerter->save_pcd_directory_ + "/MapWithoutGround.pcd";
    pcl::io::savePCDFileBinary(non_ground_file_name,  *removerter->map_global_curr_);
    ROS_INFO_STREAM("\033[1;32m save non_ground map" << non_ground_file_name  << "\033[0m");
    std::string ground_file_name = removerter->save_pcd_directory_ + "/Ground.pcd";
    pcl::io::savePCDFileBinary(ground_file_name, *removerter->map_global_static);
    ROS_INFO_STREAM("\033[1;32m save ground map" << ground_file_name  << "\033[0m");
    /// 这样做是为了减少改动removert中的变量的名称
    removerter->scans_.clear();
    removerter->scans_ = obstacle_cloud_vector;
}




