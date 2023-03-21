//
// Created by zhuhongwei on 2023/3/6.
//

#include <removert/Lidar2BEV.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/common.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


bool SaveBEVImage = false;

CloudFilter::CloudFilter() {
    max_expected_intensity_ = 1.0f;
}

CloudFilter::CloudFilter(Removerter *removerterPtr) {
    max_expected_intensity_ = 1.0f;
    RemoverterPtr = removerterPtr;
}

CloudFilter::CloudFilter(pcl::PointCloud<PointType>::Ptr input_cloud) {
    max_expected_intensity_ = 1.0f;
    setInputCloud(input_cloud);
}

CloudFilter::~CloudFilter() {
    cloud_ptr_.reset();
}

void CloudFilter::setInputCloud(pcl::PointCloud<PointType>::Ptr input_cloud) {
    cloud_ptr_.reset();
    cloud_ptr_ = input_cloud;
}

/* Return true if the point is inside the camera FOV. Used to filter points in filterFOV */
bool CloudFilter::pointInCameraFov(PointType p, double horizontal_fov) {
    // Translate the point to the camera frame
    p.x -= velo_cam_transform_.getOrigin().x();
    p.y -= velo_cam_transform_.getOrigin().y();

    // Automatically remove points behind the car or too far
    if (p.x < 0 || p.x > 100) return false;

    //double angle = (p.x != 0.0) ? std::atan(p.y / p.x) : 0.0;
    //return (abs(angle) < (horizontal_fov / 2.0));

    // Instead of computing the atan, just compare the relation between x/y
    return (abs(p.y) < (horizontal_fov / 90.0) * p.x);
}

/* Remove the points that are not in the camera FOV */
void CloudFilter::filterFOV(double horizontal_fov) {
    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&](const pcl::PointXYZI &p) {
                                                return !pointInCameraFov(p, horizontal_fov);
                                            }),
                             cloud_ptr_->points.end()
    );
    cloud_ptr_->width = cloud_ptr_->points.size();
}

/* Remove all points above the given intensity threshold */
bool CloudFilter::filterGround(PointType p, int grid_dim, const std::vector<std::vector<float> > &min,
                               const std::vector<std::vector<float> > &max,
                               const std::vector<std::vector<float> > &init, const double &height_threshold,
                               const double &cell_size) {
    ROS_INFO_STREAM("\033[1;32m --start remove point  ********" "\033[0m");
    int x = ((grid_dim / 2) + p.x / cell_size);
    int y = ((grid_dim / 2) + p.y / cell_size);
    if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim && init[x][y])
        return ((max[x][y] - min[x][y] < height_threshold));
    else {
        return false;
    }
}

/* Remove the floor points using a heightmap algorithm */
void CloudFilter::removeFloor(double cell_size, double height_threshold, int grid_dim) {

    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    std::vector<std::vector<float> > min;
    min.resize(grid_dim, std::vector<float>(grid_dim, 0));
    std::vector<std::vector<float> > max;
    max.resize(grid_dim, std::vector<float>(grid_dim, 0));
    std::vector<std::vector<float> > init;
    init.resize(grid_dim, std::vector<float>(grid_dim, 0));

    // build height map
    for (unsigned i = 0; i < cloud_ptr_->points.size(); ++i) {
        int x = ((grid_dim / 2) + cloud_ptr_->points[i].x / cell_size);
        int y = ((grid_dim / 2) + cloud_ptr_->points[i].y / cell_size);
        if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim) {
            if (!init[x][y]) {
                min[x][y] = cloud_ptr_->points[i].z;
                max[x][y] = cloud_ptr_->points[i].z;
                init[x][y] = true;
            } else {
                min[x][y] = std::min(min[x][y], cloud_ptr_->points[i].z);
                max[x][y] = std::max(max[x][y], cloud_ptr_->points[i].z);
            }
        }
    }

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&](const pcl::PointXYZI &p) {
                                                return filterGround(p, grid_dim, min, max, init, height_threshold,
                                                                    cell_size);
                                            }),
                             cloud_ptr_->points.end()
    );
    cloud_ptr_->width = cloud_ptr_->points.size();

    NonGroundScanPoints.push_back(cloud_ptr_);
}


inline float CloudFilter::rad2deg(float radians) {
    return radians * 180.0 / M_PI;
}

void CloudFilter::removeFloor(Eigen::Matrix4d scan_pose) {

    std::vector<int> AllGroundPointIdxEachScan;

    int kNumRimgRow = kFOV.first * Hres;
    int kNumRimgCol = kFOV.second * Hres;
    int lower_bound_row_idx{0};
    int lower_bound_col_idx{0};
    int upper_bound_row_idx{kNumRimgRow - 1};
    int upper_bound_col_idx{kNumRimgCol - 1};
    // row and col
    std::vector<std::vector<float> > min;
    min.resize(kNumRimgRow, std::vector<float>(kNumRimgCol, 0));

    std::vector<std::vector<float> > max;
    max.resize(kNumRimgRow, std::vector<float>(kNumRimgCol, 0));
    std::vector<std::vector<float> > init;
    init.resize(kNumRimgRow, std::vector<float>(kNumRimgCol, 0));
    // row col and pt_idx
    std::vector<std::vector<std::vector<int>>> PtIdForThisRowAndCol;
//    PtIdForThisRowAndCol.resize(kNumRimgRow, std::vector<std::vector<int> >(kNumRimgCol, std::vector<int>(
//            cloud_ptr_->points.size())));

    PtIdForThisRowAndCol.resize(kNumRimgRow, std::vector<std::vector<int> >(kNumRimgCol, std::vector<int>()));
    cv::Mat image(kNumRimgCol, kNumRimgRow, CV_8UC3, cv::Scalar(255, 255, 255));


    for (int pt_idx = 0; pt_idx < cloud_ptr_->points.size(); ++pt_idx) {
//        ROS_INFO_STREAM("\033[1;32m --  process" << pt_idx << "\033[0m");
        PointType this_point = cloud_ptr_->points[pt_idx];
        SphericalPoint sph_point{
                std::atan2(this_point.y, this_point.x),
                std::atan2(this_point.z, std::sqrt(this_point.x * this_point.x + this_point.y * this_point.y)),
                std::sqrt(this_point.x * this_point.x + this_point.y * this_point.y + this_point.z * this_point.z)
        };

        int pixel_idx_row = int(std::min(std::max(
                std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV / float(2.0))) / (kVFOV - float(0.0)))),
                float(lower_bound_row_idx)), float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(std::max(
                std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV / float(2.0))) / (kHFOV - float(0.0)))),
                float(lower_bound_col_idx)), float(upper_bound_col_idx)));



        image.at<uchar>(pixel_idx_col,pixel_idx_row) = std::min(int(this_point.z), 255);



        if (!init[pixel_idx_row][pixel_idx_col]) {
            min[pixel_idx_row][pixel_idx_col] = cloud_ptr_->points[pt_idx].z;
            max[pixel_idx_row][pixel_idx_col] = cloud_ptr_->points[pt_idx].z;
            init[pixel_idx_row][pixel_idx_col] = true;
            PtIdForThisRowAndCol[pixel_idx_row][pixel_idx_col].push_back(pt_idx);
        }
        else {
            min[pixel_idx_row][pixel_idx_col] = std::min(min[pixel_idx_row][pixel_idx_col],
                                                         cloud_ptr_->points[pt_idx].z);
            max[pixel_idx_row][pixel_idx_col] = std::max(max[pixel_idx_row][pixel_idx_col],
                                                         cloud_ptr_->points[pt_idx].z);
            PtIdForThisRowAndCol[pixel_idx_row][pixel_idx_col].push_back(pt_idx);
        }
    }

    // 小于阈值是地面点,循环找出所有的地面点
    for (int i = 0; i <= kNumRimgRow-1; ++i) {
        for (int j = 0; j <= kNumRimgCol-1; ++j)
            if (init[i][j]){
                if (max[i][j] - min[i][j] < HeightThreshold) {
                    std::vector<int> PtIdsVector = PtIdForThisRowAndCol[i][j];
                    AllGroundPointIdxEachScan.insert(AllGroundPointIdxEachScan.end(), PtIdsVector.begin(), PtIdsVector.end());
                }
            }
    }

    /// extractor static and dynamic
    pcl::PointCloud<PointType>::Ptr ground(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr landscape(new pcl::PointCloud<PointType>);
    pcl::ExtractIndices<PointType> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(AllGroundPointIdxEachScan);
    extractor.setInputCloud(cloud_ptr_);
    extractor.setIndices(index_ptr);
    extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index
    extractor.filter(*ground);
    // save
    extractor.setNegative(true);
    extractor.filter(*landscape);
    /// save non-ground scan
    ObstacleCloudVector.push_back(landscape);

    /// save ground as static directly in map_global_static map
    pcl::PointCloud<PointType>::Ptr scan_ground_global_coord(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*ground, *scan_ground_global_coord, kSE3MatExtrinsicLiDARtoPoseBase); /// 将点云从雷达坐标系变换到位姿统一（kitty就是以相机的坐标为基准）的坐标系下
    pcl::transformPointCloud(*scan_ground_global_coord, *scan_ground_global_coord, scan_pose); // 变换到第一针坐标系下
    *(RemoverterPtr->map_global_static) += *scan_ground_global_coord;
    scan_ground_global_coord->clear();

    ///merge non-ground points as map
    pcl::PointCloud<PointType>::Ptr scan_landscape_global_coord(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*landscape, *scan_landscape_global_coord, kSE3MatExtrinsicLiDARtoPoseBase); /// 将点云从雷达坐标系变换到位姿统一（kitty就是以相机的坐标为基准）的坐标系下
    pcl::transformPointCloud(*scan_landscape_global_coord, *scan_landscape_global_coord, scan_pose); // 变换到第一针坐标系下
    /// map with non-ground and fed to remove stage
    *(RemoverterPtr->map_global_curr_) += *scan_landscape_global_coord;

//    std::string non_ground_file_name = save_pcd_directory_ + "/MapWithoutGround.pcd";
//    ROS_INFO_STREAM("\033[1;32m save non_ground map" << non_ground_file_name  << "\033[0m");
//    pcl::io::savePCDFileBinary(non_ground_file_name,  *landscape);
//
//    std::string ground_file_name = save_pcd_directory_ + "/Ground.pcd";
//    ROS_INFO_STREAM("\033[1;32m save ground map" << ground_file_name << "\033[0m");
//    pcl::io::savePCDFileBinary(ground_file_name, *ground);

    landscape->clear();
    ground->clear();
}


std::shared_ptr<cv::Mat> CloudFilter::birdGround(double bv_cell_size, int ground_cell_span, double grid_dim) {
    int grid_cells = grid_dim / bv_cell_size; // Number of col/rows of the birdview
    int ground_cells = floor(grid_cells / ground_cell_span); // Number of cols/rows of the auxiliary ground bird view
    std::shared_ptr<cv::Mat> bird_ground(new cv::Mat(grid_cells, grid_cells, CV_32FC1));
    std::shared_ptr<cv::Mat> small_ground(new cv::Mat(ground_cells, ground_cells, CV_32FC1));
    std::shared_ptr<cv::Mat> median_ground(new cv::Mat(ground_cells, ground_cells, CV_32FC1));
    float **aux_ground = new float *[ground_cells];

    // Allocate memory
    for (int i = 0; i < ground_cells; ++i) {
        aux_ground[i] = new float[ground_cells];
    }

    // Init the grids
    for (int i = 0; i < ground_cells; ++i) {
        for (int j = 0; j < ground_cells; ++j) {
            aux_ground[i][j] = 9999.9;
        }
    }

    // Fill the aux grid
    for (const auto &point: cloud_ptr_->points) {
        if (point.z < -3.0) continue;
        float z = point.z + base_velo_transform_.getOrigin().z();
        int x = grid_cells / 2 - point.x / bv_cell_size;
        int y = grid_cells / 2 - point.y / bv_cell_size;

        int x_ground = (double) x / ground_cell_span - .5f;
        int y_ground = (double) y / ground_cell_span - .5f;

        // Store min value of pixels belonging to the area
        if (x_ground >= 0 && x_ground < ground_cells && y_ground >= 0 && y_ground < ground_cells) {
            // Update cell max height
            aux_ground[x_ground][y_ground] = std::min(aux_ground[x_ground][y_ground], z);
        }
    }

    // Hack to fill points near and below the car (5x5 area centered on the lidar)
    for (int i = 0; i < ground_cells; ++i) {
        for (int j = 0; j < ground_cells; ++j) {
            float z = aux_ground[i][j];
            if (fabs(z) > 0.2 && fabs((i * ground_cell_span - grid_cells / 2.) * bv_cell_size) < 5. &&
                fabs((j * ground_cell_span - grid_cells / 2.) * bv_cell_size) < 5.) {
                aux_ground[i][j] = 0;
            }
        }
    }
    // Copy the aux grid to mat
    for (int i = 0; i < ground_cells; ++i) {
        float *row_ptr = small_ground->ptr<float>(i);
        for (int j = 0; j < ground_cells; ++j) {
            *row_ptr++ = (float) aux_ground[i][j];
        }
    }

    // Apply median filter
    medianBlur(*small_ground, *median_ground, 3);

    // Fill the bird view mat
    for (int i = 0; i < grid_cells; ++i) {
        float *row_ptr = bird_ground->ptr<float>(i);
        for (int j = 0; j < grid_cells; ++j) {
            int x_ground = (double) i / ground_cell_span - .5f; // TODO Review grid access
            int y_ground = (double) j / ground_cell_span - .5f;
            *row_ptr++ = median_ground->at<float>(x_ground, y_ground);
        }
    }

    for (int i = 0; i < ground_cells; ++i) {
        delete[] aux_ground[i];
    }
    delete[] aux_ground;

    return bird_ground;
}

std::shared_ptr<cv::Mat>
CloudFilter::birdView(double cell_size, double max_height, int num_slices, double grid_dim, bool sample_points) {
    // TODO: Add an interpolation to fill the unknown cells

    int grid_cells = grid_dim / cell_size;

    int num_channels = 3 + num_slices;

    // The channels are: intensity, min_height, max_height, density per slice.
    std::shared_ptr<cv::Mat> bird_view(new cv::Mat(grid_cells, grid_cells, CV_8UC(num_channels)));
    //std::cout << bird_view->rows << " " << bird_view->cols << " " << bird_view->channels() << " " << CV_8UC(num_channels) << std::endl;

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);


    int ***density = new int **[grid_cells];
    float ***height = new float **[grid_cells];
    float **intensity = new float *[grid_cells];

    for (int i = 0; i < grid_cells; ++i) {
        density[i] = new int *[grid_cells];
        height[i] = new float *[grid_cells];
        intensity[i] = new float[grid_cells];

        for (int j = 0; j < grid_cells; ++j) {
            density[i][j] = new int[num_slices];
        }
        for (int j = 0; j < grid_cells; ++j) {
            height[i][j] = new float[2];
        }
    }

    // Init the grids
    for (int i = 0; i < grid_cells; ++i) {
        for (int j = 0; j < grid_cells; ++j) {
            for (int k = 0; k < num_slices; ++k) {
                density[i][j][k] = 0;
            }

            height[i][j][0] = 9999.9;
            height[i][j][1] = -9999.9;
            intensity[i][j] = 0.0;
        }
    }

    // Fill the grids
    for (const auto &point: cloud_ptr_->points) {
        float z = point.z + base_velo_transform_.getOrigin().z();
        if (z < max_height) {
            int x = grid_cells / 2 - point.x / cell_size;
            int y = grid_cells / 2 - point.y / cell_size;

            if (x >= 0 && x < grid_cells && y >= 0 && y < grid_cells) {
                bool chosen = true;
                if (chosen) {
                    // Update cell min height
                    height[x][y][0] = std::min(height[x][y][0], z);
                    // Update cell max height
                    height[x][y][1] = std::max(height[x][y][1], z);
                    // Increment the number of points in the cell
                    for (int k = 0; k < num_slices; k++) {
                        if (z > k * max_height / num_slices and z < (k + 1) * max_height / num_slices)
                            density[x][y][k]++;
                    }
                    // Update the cell intensity sum to later compute the mean
                    intensity[x][y] += point.intensity / max_expected_intensity_;

                }
            }
        }
    }

    // Fill the bird view mat
    uchar *mat_ptr = bird_view->data;
    for (int i = 0; i < grid_cells; ++i) {
        for (int j = 0; j < grid_cells; ++j) {
            // Set unknown heights (still in -9999.9) to 0
            height[i][j][0] = std::max(height[i][j][0], (float) 0.0);
            height[i][j][1] = std::max(height[i][j][1], (float) 0.0);
            // Normalize the height to 0-255
            *mat_ptr++ = (uchar) (255 * height[i][j][0] / max_height);
            *mat_ptr++ = (uchar) (255 * height[i][j][1] / max_height);

            int cell_density = 0;
            for (int k = 0; k < num_slices; ++k) {
                // Limit the density to 255 (we should never have that many points in the same cell...)
                int norm_factor = max_points_map_[i][j][k];

                cell_density += density[i][j][k];
                //              if(sample_points){
                //                norm_factor = std::min(norm_factor, goal_max_points);
                //              }
                int points_cell_pixel = (float) density[i][j][k] / norm_factor * 255;
                *mat_ptr++ = (uchar) std::min(points_cell_pixel, 255);
            }

            // Compute the intensity mean for that cell given the cell density and normalize it to 0-255
            float norm_intensity = 0;
            norm_intensity = (cell_density > 0) ? std::min(255.f, (255 * intensity[i][j] / cell_density)) : 0;

            *mat_ptr++ = (uchar) norm_intensity;
        }
    }

    for (int i = 0; i < grid_cells; ++i) {
        for (int j = 0; j < grid_cells; ++j) {
            delete[] density[i][j];
            delete[] height[i][j];
        }
    }

    for (int i = 0; i < grid_cells; ++i) {
        delete[] density[i];
        delete[] height[i];
        delete[] intensity[i];
    }
    delete[] density;
    delete[] height;
    delete[] intensity;

    return bird_view;
}

/* Remove all points above the given intensity threshold */
void CloudFilter::filterIntensities(double intensity_threshold) {
    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&intensity_threshold](const pcl::PointXYZI &p) {
                                                return p.intensity > intensity_threshold;
                                            }),
                             cloud_ptr_->points.end()
    );
    cloud_ptr_->width = cloud_ptr_->points.size();
}


/* Wait for the transform lidar -> camera and update velo_cam_transform_ */
void CloudFilter::initTF(std::string lidar_frame, std::string camera_frame) {
    if (!tf_) tf_ = new tf::TransformListener;
    bool tf_error = true;
    while (tf_error) {
        try {
            tf_->waitForTransform(lidar_frame, camera_frame, ros::Time(0), ros::Duration(5));
            tf_->lookupTransform(lidar_frame, camera_frame, ros::Time(0), velo_cam_transform_);
            tf_->waitForTransform("base_footprint", lidar_frame, ros::Time(0), ros::Duration(5));
            tf_->lookupTransform("base_footprint", lidar_frame, ros::Time(0), base_velo_transform_);
            tf_error = false;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("%s", ex.what());
            //throw (ex);
        }
    }
    std::cout << "New transform: " << velo_cam_transform_.getOrigin().x() << ", " << velo_cam_transform_.getOrigin().y()
              << std::endl;
}

//void CloudFilter::setVeloToCamTransform(tf::StampedTransform velo_cam_transform) {
//    velo_cam_transform_ = velo_cam_transform;
//}
//
//void CloudFilter::setVeloToBaseTransform(tf::StampedTransform base_velo_transform) {
//    base_velo_transform_ = base_velo_transform;
//}


void CloudFilter::initMaxPointsMap(int grid_dim, float cell_size, float z_min, float z_max, int num_slices, int planes,
                                   float low_angle, float h_res, float v_res) {
    std::stringstream map_path;
//    map_path << ros::package::getPath("lidar_bev") << "/maps/";
    std::stringstream velo_h;
    velo_h << std::fixed;
    velo_h << std::setprecision(2);
    velo_h << base_velo_transform_.getOrigin().z();
    std::stringstream python_cmd;
//    python_cmd << ros::package::getPath("lidar_bev") << "/scripts/max_points_map.py";
    python_cmd << " --maps " << map_path.str();
    python_cmd << " --map_size " << grid_dim;
    python_cmd << " --cell_size " << cell_size;
    python_cmd << " --min_height " << z_min;
    python_cmd << " --max_height " << z_max;
    python_cmd << " --num_slices " << num_slices;
    python_cmd << " --num_planes " << planes;
    python_cmd << " --velo_minangle " << low_angle;
    python_cmd << " --velo_hres " << h_res;
    python_cmd << " --velo_vres " << v_res;
    python_cmd << " --velo_height " << velo_h.str();
    std::cout << "Required max_points map not found, creating map..." << std::endl;
    std::cout << python_cmd.str() << std::endl;
    system(python_cmd.str().c_str());

    // Resize the matrix
    int grid_cells = grid_dim / cell_size;
    max_points_map_.resize(grid_cells);
    for (int i = 0; i < max_points_map_.size(); ++i) {
        max_points_map_[i].resize(grid_cells);
        for (int j = 0; j < max_points_map_[i].size(); ++j) {
            max_points_map_[i][j].resize(num_slices);
        }
    }

    for (int n = 0; n < num_slices; ++n) {
        std::stringstream file_name;
        file_name << std::fixed;
        file_name << std::setprecision(2);
        file_name << map_path.str() << grid_dim << "_";
        file_name << cell_size << "_";
        file_name << planes << "_";
        file_name << velo_h.str();
        file_name << "_slice" << n << "_map.txt";

        std::ifstream f(file_name.str());
        if (!f.good()) {
            std::cout << "[ERROR] Cannot read " << file_name.str() << ". Exiting..." << std::endl;
            exit(-1);
        }

        // Load map from file
        std::string line;
        int i = 0, j = 0;

        while (std::getline(f, line)) {
            float value;
            int j = 0;
            std::stringstream ss(line);
            while (ss >> value) {
                max_points_map_[i][j][n] = value;
                ++j;
            }
            ++i;
        }
        f.close();
    }


//    } else {
//      std::cout << "Compute max_points with its corresponding python script and run this node again" << std::endl;
//      exit(-1);
//    }

    /*std::cout << "Computing max points map..." << std::endl;
    float cos_h_res = cos(h_res);
    ros::Time init = ros::Time::now();
    int num_plane = 0;
    float x,y;
    //std::ofstream myfile;
    //myfile.open ("/home/lsi2/catkin_ws/src/didi_challenge/src/example.txt");
    int grid_cells = grid_dim / cell_size;
    std::cout << "Grid cells: " << grid_cells << ". Velodyne height: " << base_velo_transform_.getOrigin().z() << std::endl;

    // Resize the matrix
    max_points_map_.resize(grid_cells);
    for (int i = 0; i < max_points_map_.size(); ++i)
    {
        max_points_map_[i].resize(grid_cells);
    }

    for (int i = 0; i < grid_cells; i++)
    {
        x = grid_cells * cell_size / 2 - i * cell_size;
        for (int j = 0; j < grid_cells; j++)
        {
            y = grid_cells * cell_size / 2 - j * cell_size;
            if (j == 0 && i == 0)
            {
                max_points_map_[i][j] = 1;
            }
            else
            {
                float distance = pow((pow(x, 2) + pow(y, 2)), 0.5); //get squared distance
                float dispersion = pow(2 * pow(distance, 2) * (1 - cos_h_res), 0.5);

                float angle = atan(base_velo_transform_.getOrigin().z() / distance) * 180 / M_PI;

                num_plane = abs((low_opening - angle) / v_res);
                float max_points = (planes - num_plane) * cell_size / dispersion;
                max_points_map_[i][j] = max_points / 2; //limit it to a % of the max points possibles,
                if (max_points_map_[i][j] < 1)
                {
                    max_points_map_[i][j] = 1;
                }
            }
        }
    }
    std::cout << "Time consumed building the max points map: " << ros::Time::now() - init << std::endl;*/


}

/**
 * transform point cloud to BEV image, then remove the ground, final save the image
 */
std::vector<pcl::PointCloud<PointType>::Ptr> CloudFilter::RunLidar2BEVAndRemove() {
    for (std::size_t idx_scan = 0; idx_scan < RemoverterPtr->scans_.size(); ++idx_scan) {

        pcl::PointCloud<PointType>::Ptr scan = RemoverterPtr->scans_.at(idx_scan);
        Eigen::Matrix4d scan_pose = RemoverterPtr->scan_poses_.at(idx_scan);
        ROS_INFO_STREAM("\033[1;32m -- start change to BEV ********" "\033[0m");
        setInputCloud(scan);

        /* Remove the points that are not in the camera FOV */
//        filterFOV(CameraFov);
//        removeFloor(CellSize, HeightThreshold, int GridDim);
//        test();

        removeFloor(scan_pose);

        ROS_INFO_STREAM("\033[1;32m --removeFloor finsh ********" "\033[0m");

        if (SaveBEVImage) {
//            initMaxPointsMap(GridDim, CellSize, 0, 5, 3, 64, 24.9, Hres, Vres);
            std::shared_ptr<cv::Mat> bird_view = birdView(CellSize, MaxHeight, 3, GridDim);
            cv::Mat final_birdview = bird_view->clone();

            // Crop to desired region
            // if(grid_min_x!=MIN_CROP || grid_min_y!=MIN_CROP || grid_max_x!=MAX_CROP || grid_max_y!=MAX_CROP){
            assert(GridMinX >= -GridDim / 2.);
            assert(GridMinY >= -GridDim / 2.);
            assert(GridMinX <= GridDim / 2.);
            assert(GridMaxY <= GridDim / 2.);
            assert(GridMinY < GridMaxY);
            assert(GridMinX < GridMaxX);

            int x_max = final_birdview.rows / 2. - GridMinX / CellSize;
            int y_min = final_birdview.cols / 2. + GridMinY / CellSize;

            int h_pixels = (GridMaxX - GridMinX) / CellSize;
            int w_pixels = (GridMaxY - GridMinY) / CellSize;

            int x_min = x_max - h_pixels;

            final_birdview = final_birdview(cv::Rect(y_min, x_min, w_pixels, h_pixels));

            std::shared_ptr<cv::Mat> bird_ground = birdGround(CellSize, GroundCellSpan, GridDim);
            cv::Mat final_birdground = bird_ground->clone();

            final_birdground = final_birdground(cv::Rect(y_min, x_min, w_pixels, h_pixels));

            std::vector<cv::Mat> channels;
            cv::split(final_birdview, channels);
            if (channels.size() == 3) {
                // Single png image
                cv::Mat three_ch;
                cv::merge(channels, three_ch);
                std::string saving_absolute =
                        RemoverterPtr->save_pcd_directory_ + "/BEVImage" + std::to_string(idx_scan) + ".png";
                cv::imwrite(saving_absolute, three_ch);
            }
        }
    }
    RemoverterPtr->scans_ = ObstacleCloudVector;
    ROS_INFO_STREAM("\033[1;32m -- END chage to BEV ********" "\033[0m");
    return NonGroundScanPoints;
}


void CloudFilter::test(){

    for (std::size_t idx_scan = 0; idx_scan < RemoverterPtr->scans_.size(); ++idx_scan) {

        pcl::PointCloud<PointType>::Ptr cloud = RemoverterPtr->scans_.at(idx_scan);


        float min_x = cloud->points[0].x;
        float max_x = cloud->points[0].x;
        float min_y = cloud->points[0].y;
        float max_y = cloud->points[0].y;

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            if (cloud->points[i].x < min_x) min_x = cloud->points[i].x;
            if (cloud->points[i].x > max_x) max_x = cloud->points[i].x;
            if (cloud->points[i].y < min_y) min_y = cloud->points[i].y;
            if (cloud->points[i].y > max_y) max_y = cloud->points[i].y;
        }

        // Determine the size of the image
        int img_width = static_cast<int>((max_x - min_x) * 10);
        int img_height = static_cast<int>((max_y - min_y) * 10);

        // Create the image
        cv::Mat image(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));

        // Draw the point cloud on the image
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            int img_x = static_cast<int>(abs((cloud->points[i].x - min_x)) * 10);
            int img_y = static_cast<int>(abs((cloud->points[i].y - min_y)) * 10);
//        if (img_x>=0 && img_x< img_width && img_y>0 && img_y<img_height){
            image.at<uchar>(img_y,img_x) = std::min(int(cloud->points[i].z), 255);
//        }
        }

        // Save the image to file
        cv::imwrite("/home/zhuhongwei/Removert_sci4/src/removert/test.png", image);

    }

//    std::vector<int> AllGroundPointIdxEachScan;
//
//    int kNumRimgRow = 400;
//    int kNumRimgCol =400;
//    int lower_bound_row_idx{0};
//    int lower_bound_col_idx{0};
//    int upper_bound_row_idx{kNumRimgRow - 1};
//    int upper_bound_col_idx{kNumRimgCol - 1};
//    // row and col
//    std::vector<std::vector<float> > min;
//    min.resize(kNumRimgRow, std::vector<float>(kNumRimgCol, 0));
//
//    std::vector<std::vector<float> > max;
//    max.resize(kNumRimgRow, std::vector<float>(kNumRimgCol, 0));
//    std::vector<std::vector<float> > init;
//    init.resize(kNumRimgRow, std::vector<float>(kNumRimgCol, 0));
//    // row col and pt_idx
//    std::vector<std::vector<std::vector<int>>> PtIdForThisRowAndCol;
////    PtIdForThisRowAndCol.resize(kNumRimgRow, std::vector<std::vector<int> >(kNumRimgCol, std::vector<int>(
////            cloud_ptr_->points.size())));
//
//    PtIdForThisRowAndCol.resize(kNumRimgRow, std::vector<std::vector<int> >(kNumRimgCol, std::vector<int>()));
//    cv::Mat image(kNumRimgCol, kNumRimgRow, CV_8UC3, cv::Scalar(255, 255, 255));
//
//
//    for (int pt_idx = 0; pt_idx < cloud_ptr_->points.size(); ++pt_idx) {
////        ROS_INFO_STREAM("\033[1;32m --  process" << pt_idx << "\033[0m");
//        PointType this_point = cloud_ptr_->points[pt_idx];
//        SphericalPoint sph_point{
//                std::atan2(this_point.y, this_point.x),
//                std::atan2(this_point.z, std::sqrt(this_point.x * this_point.x + this_point.y * this_point.y)),
//                std::sqrt(this_point.x * this_point.x + this_point.y * this_point.y + this_point.z * this_point.z)
//        };
//
//        int pixel_idx_row = int(std::min(std::max(
//                std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV / float(2.0))) / (kVFOV - float(0.0)))),
//                float(lower_bound_row_idx)), float(upper_bound_row_idx)));
//        int pixel_idx_col = int(std::min(std::max(
//                std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV / float(2.0))) / (kHFOV - float(0.0)))),
//                float(lower_bound_col_idx)), float(upper_bound_col_idx)));
//
//
//
//        image.at<uchar>(pixel_idx_col,pixel_idx_row) = std::min(int(this_point.z), 255);
//
//
//
//        if (!init[pixel_idx_row][pixel_idx_col]) {
//            min[pixel_idx_row][pixel_idx_col] = cloud_ptr_->points[pt_idx].z;
//            max[pixel_idx_row][pixel_idx_col] = cloud_ptr_->points[pt_idx].z;
//            init[pixel_idx_row][pixel_idx_col] = true;
//            PtIdForThisRowAndCol[pixel_idx_row][pixel_idx_col].push_back(pt_idx);
//        }
//        else {
//            min[pixel_idx_row][pixel_idx_col] = std::min(min[pixel_idx_row][pixel_idx_col],
//                                                         cloud_ptr_->points[pt_idx].z);
//            max[pixel_idx_row][pixel_idx_col] = std::max(max[pixel_idx_row][pixel_idx_col],
//                                                         cloud_ptr_->points[pt_idx].z);
//            PtIdForThisRowAndCol[pixel_idx_row][pixel_idx_col].push_back(pt_idx);
//        }
//    }
//
//    cv::imwrite("/home/zhuhongwei/Removert_sci4/src/removert/test.png", image);




}
