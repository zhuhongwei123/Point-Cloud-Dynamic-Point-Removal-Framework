#pragma once

#include "removert/utility.h"

class RosNodeHandle
{
public:
    ros::NodeHandle nh_super;
}; // class: RosNodeHandle


class RosParamServer: public RosNodeHandle
{
public:
    // 
    ros::NodeHandle & nh;

    //
    std::string pointcloud_topic;

    // removert params 
    float kVFOV; // 激光雷达的FOV
    float kHFOV;
    std::pair<float, float> kFOV;

    // sequence info 
    std::vector<double> kVecExtrinsicLiDARtoPoseBase; // 旋转T,这个是std的vecvtor的表达方式
    Eigen::Matrix4d kSE3MatExtrinsicLiDARtoPoseBase; // 旋转T，比如雷达对相机，我们的系统是雷达对于IMU
        // Base is where of the pose writtened (e.g., for KITTI, poses is usually in camera)
        // if the pose file is obtained via lidar odometry itself, then kMatExtrinsicLiDARtoBase is eye(4)
    Eigen::Matrix4d kSE3MatExtrinsicPoseBasetoLiDAR;

    // sequence bin files
    bool isScanFileKITTIFormat_;

    std::string sequence_scan_dir_;
    std::vector<std::string> sequence_scan_names_;
    std::vector<std::string> sequence_scan_paths_;
    int num_total_scans_of_sequence_;
    float kDownsampleVoxelSize; // downsample_voxel_size

    // sequence pose file
    std::string sequence_pose_path_; // 位姿txt的路径
    std::vector<Eigen::Matrix4d> sequence_scan_poses_;
    std::vector<Eigen::Matrix4d> sequence_scan_inverse_poses_; // used for global to local

    // target region to removerting 
    int start_idx_; // yaml 里面参数 start_idx
    int end_idx_;

    bool use_keyframe_gap_; // yaml
    bool use_keyframe_meter_; // yaml
    int keyframe_gap_;// yaml
    float keyframe_gap_meter_;// yaml

    // 
    std::vector<float> remove_resolution_list_;
    std::vector<float> revert_resolution_list_;

    // 
    int kNumOmpCores; // yaml里面的num_omp_cores

    //
    pcl::PointCloud<PointType>::Ptr single_scan;
    pcl::PointCloud<PointType>::Ptr projected_scan;

    // ros pub
    ros::Publisher scan_publisher_;
    ros::Publisher global_scan_publisher_;

    ros::Publisher original_map_local_publisher_;

    ros::Publisher curr_map_local_publisher_;
    ros::Publisher static_map_local_publisher_;
    ros::Publisher dynamic_map_local_publisher_;

    ros::Publisher static_curr_scan_publisher_;
    ros::Publisher dynamic_curr_scan_publisher_;

    float rimg_color_min_; // For visualization of range images (rviz -d removert_visualization.rviz)
    float rimg_color_max_; //  For visualization of range images (rviz -d removert_visualization.rviz)
    std::pair<float, float> kRangeColorAxis; // meter
    std::pair<float, float> kRangeColorAxisForDiff; // meter 

    image_transport::ImageTransport ROSimg_transporter_;

    sensor_msgs::ImagePtr scan_rimg_msg_;
    image_transport::Publisher scan_rimg_msg_publisher_;
    image_transport::Publisher Revert_scan_rimg_msg_publisher_;
    sensor_msgs::ImagePtr map_rimg_msg_;
    image_transport::Publisher map_rimg_msg_publisher_;
    image_transport::Publisher Revert_map_rimg_msg_publisher_;

    sensor_msgs::ImagePtr diff_rimg_msg_;
    image_transport::Publisher diff_rimg_msg_publisher_;
    image_transport::Publisher Revert_diff_rimg_msg_publisher_;
    sensor_msgs::ImagePtr map_rimg_ptidx_msg_;
    image_transport::Publisher map_rimg_ptidx_msg_publisher_;
    image_transport::Publisher Revert_map_rimg_ptidx_msg_publisher_;
    //
    bool kFlagSaveMapPointcloud;
    bool kFlagSaveCleanScans;
    std::string save_pcd_directory_; //PCDdata

    double seg_threshold_; // RANSAC DistanceThreshold
    int max_iterations_;

    // Lidar2BEV
    double CameraFov;
    double HeightThreshold;
    double MaxHeight;
    double MinHeight;
    int GridDim;
    double Hres;
    double Vres;
    double LowOpening;
    int GridDimHeightMap;
    double CellSizeHeightMap;
    int GridMinY;
    int GridMaxY;
    int GridMinX;
    int GridMaxX;
    double CellSize;
    int GroundCellSpan;


public:
    RosParamServer();

}; // class: RosParamServer
