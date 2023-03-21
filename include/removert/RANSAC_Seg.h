//
// Created by zhuhongwei on 2023/3/2.
//

#ifndef REMOVERT_RANSAC_SEG_H
#define REMOVERT_RANSAC_SEG_H

#include "removert/RosParamServer.h"

#include "Removerter.h"

#include <stdlib.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <functional>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

class RANSAC_seg{
private:
    Removerter* removerter;

    std::vector<pcl::PointCloud<PointType>::Ptr> ground_cloud_vector; // 存放地面点云的queue
    std::vector<double> ground_cloud_time_vector; // 存放地面点云time_stamp的queue

    std::vector<pcl::PointCloud<PointType>::Ptr> obstacle_cloud_vector; // 存放全部点云的vector
    std::vector<double> obstacle_cloud_time_vector; // 存放全部点云time_stamp读queue
    pcl::PointCloud<PointType>::Ptr map_ground_static_;
    pcl::PointCloud<PointType>::Ptr map_non_ground_clouds_;


public:

    RANSAC_seg(Removerter* removerter);
    ~RANSAC_seg();
    void RANSAC_segmentation(std::vector<pcl::PointCloud<PointType>::Ptr> vector_scans,std::vector<Eigen::Matrix4d> vector_poses);
    void RANSAC_segmentation();
};

#endif



