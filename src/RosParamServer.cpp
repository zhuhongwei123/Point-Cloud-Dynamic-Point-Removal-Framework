#include "removert/RosParamServer.h"


RosParamServer::RosParamServer()
: nh(nh_super), ROSimg_transporter_(nh)
{
    nh.param<bool>("removert/isScanFileKITTIFormat", isScanFileKITTIFormat_, true);

    // for visualization 
 
    scan_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/scan_rimg_single", 10);
    map_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/map_rimg_single", 10);
    diff_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/diff_rimg_single", 10);
    map_rimg_ptidx_msg_publisher_ = ROSimg_transporter_.advertise("/map_rimg_ptidx_single", 10);

    nh.param<float>("removert/rimg_color_min", rimg_color_min_, 0.0);
    nh.param<float>("removert/rimg_color_max", rimg_color_max_, 10.0);
    kRangeColorAxis = std::pair<float, float> {rimg_color_min_, rimg_color_max_}; // meter
    kRangeColorAxisForDiff = std::pair<float, float>{0.0, 0.5}; // meter  可视化页面的

    // fov 
    nh.param<float>("removert/sequence_vfov", kVFOV, 50.0);
    nh.param<float>("removert/sequence_hfov", kHFOV, 360.0);
    kFOV = std::pair<float, float>(kVFOV, kHFOV);

    // resolution 
    nh.param<std::vector<float>>("removert/remove_resolution_list", remove_resolution_list_, std::vector<float>()); // 分辨率列表
    nh.param<std::vector<float>>("removert/revert_resolution_list", revert_resolution_list_, std::vector<float>());

    // sequcne system info
    std::vector<double> Transform = {-1.857e-03, -9.999e-01, -8.039e-03, -4.784e-03,-6.481e-03,  8.0518e-03, -9.999e-01, -7.337e-02,9.999e-01, -1.805e-03, -6.496e-03, -3.339e-01, 0.0, 0.0, 0.0,1.0};
    nh.param<std::vector<double>>("removert/ExtrinsicLiDARtoPoseBase", kVecExtrinsicLiDARtoPoseBase, Transform);
    kSE3MatExtrinsicLiDARtoPoseBase = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(kVecExtrinsicLiDARtoPoseBase.data(), 4, 4);
    kSE3MatExtrinsicPoseBasetoLiDAR = kSE3MatExtrinsicLiDARtoPoseBase.inverse();

    // parsing bin file paths
    nh.param<std::string>("removert/sequence_scan_dir", sequence_scan_dir_, "/home/zhuhongwei/Downloads/data_slam/KITTY/test/bin/");
    for(auto& _entry : fs::directory_iterator(sequence_scan_dir_)) {
        sequence_scan_names_.emplace_back(_entry.path().filename());
        sequence_scan_paths_.emplace_back(_entry.path());
    }
    std::sort(sequence_scan_names_.begin(), sequence_scan_names_.end());
    std::sort(sequence_scan_paths_.begin(), sequence_scan_paths_.end());

    num_total_scans_of_sequence_ = sequence_scan_paths_.size();
    ROS_INFO_STREAM("\033[1;32m Total : " << num_total_scans_of_sequence_ << " scans in the directory.\033[0m");

    // point cloud pre-processing
    nh.param<float>("removert/downsample_voxel_size", kDownsampleVoxelSize, 0.05);

    // parsing pose info /home/zhuhongwei/Downloads/data_slam/KITTY/test/pose/08.txt
    nh.param<std::string>("removert/sequence_pose_path", sequence_pose_path_, "/home/zhuhongwei/Downloads/data_slam/KITTY/test/pose/08.txt");
    std::ifstream pose_file_handle (sequence_pose_path_);
    int num_poses {0};
    std::string strOneLine;
    while (getline(pose_file_handle, strOneLine)) 
    {
        // str to vec
        std::vector<double> ith_pose_vec = splitPoseLine(strOneLine, ' ');
        if(ith_pose_vec.size() == 12) {
            /// 补齐成为4x4的矩阵
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(1.0));
        }
    
        // vec to eig
        Eigen::Matrix4d ith_pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ith_pose_vec.data(), 4, 4);
        Eigen::Matrix4d ith_pose_inverse = ith_pose.inverse();

        // save (move)
        // cout << "Pose of scan: " << sequence_scan_names_.at(num_poses) << endl;
        // cout << ith_pose << endl;
        sequence_scan_poses_.emplace_back(ith_pose);
        sequence_scan_inverse_poses_.emplace_back(ith_pose_inverse);

        num_poses++;
    }
    // check the number of scans and the number of poses are equivalent
    assert(sequence_scan_paths_.size() == sequence_scan_poses_.size());

    // target scan index range (used in Removert.cpp)
    nh.param<int>("removert/start_idx", start_idx_, 1);
    nh.param<int>("removert/end_idx", end_idx_, 100);

    nh.param<bool>("removert/use_keyframe_gap", use_keyframe_gap_, true);
    nh.param<bool>("removert/use_keyframe_meter", use_keyframe_meter_, false);
    nh.param<int>("removert/keyframe_gap", keyframe_gap_, 10);
    nh.param<float>("removert/keyframe_meter", keyframe_gap_meter_, 2.0);

    // faster
    nh.param<int>("removert/num_omp_cores", kNumOmpCores, 4);

    // save info
    nh.param<bool>("removert/saveMapPCD", kFlagSaveMapPointcloud, false);
    nh.param<bool>("removert/saveCleanScansPCD", kFlagSaveCleanScans, false);
    nh.param<std::string>("removert/save_pcd_directory", save_pcd_directory_, "/home/zhuhongwei/Removert_sci4/src/removert/PCDData/KITTY/08");
// ransac seg
    nh.param<double>("segmentation/seg_threshold", seg_threshold_, 1.21);
    nh.param<int>("segmentation/max_iterations", max_iterations_, 5);
// Lidar2BEV
    nh.param<double>("Lidar2BEVAndRemovePara/camera_fov", CameraFov, 110);
    nh.param<double>("Lidar2BEVAndRemovePara/height_threshold", HeightThreshold, 0.1);
    nh.param<double>("Lidar2BEVAndRemovePara/max_height", MaxHeight, 3.0);
    nh.param<double>("Lidar2BEVAndRemovePara/min_height", MinHeight, -1.0);
    nh.param<int>("Lidar2BEVAndRemovePara/grid_dim", GridDim, 10);
    nh.param<double>("Lidar2BEVAndRemovePara/h_res", Hres, 0.2);
    nh.param<double>("Lidar2BEVAndRemovePara/v_res", Vres, 0.4);
    nh.param<double>("Lidar2BEVAndRemovePara/low_opening", LowOpening, 24.9);
    nh.param<int>("Lidar2BEVAndRemovePara/grid_dim_height_map", GridDimHeightMap, 300);
    nh.param<double>("Lidar2BEVAndRemovePara/cell_size_height_map", CellSizeHeightMap, 0.25);
    nh.param<int>("Lidar2BEVAndRemovePara/grid_min_y", GridMinY, -70); // enable non-square BEV images by cropping on the left
    nh.param<int>("Lidar2BEVAndRemovePara/grid_max_y", GridMaxY, 70); // enable non-square BEV images by cropping on the right
    nh.param<int>("Lidar2BEVAndRemovePara/grid_min_x", GridMinX, -70); // enable non-square BEV images by cropping on the bottom
    nh.param<int>("Lidar2BEVAndRemovePara/grid_max_x", GridMaxX, 70); // enable non-square BEV images by cropping on the top
    nh.param<double>("Lidar2BEVAndRemovePara/cell_size", CellSize, 0.1); // enable non-square BEV images by cropping on the top
    nh.param<int>("Lidar2BEVAndRemovePara/ground_cell_span", GroundCellSpan, 40); // enable non-square BEV images by cropping on the top

    usleep(100);
} // ctor RosParamServer

