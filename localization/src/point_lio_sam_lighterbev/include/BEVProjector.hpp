#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>  // 用于 pcl::VoxelGrid
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omp.h>


namespace std {
    template <>
    struct hash<Eigen::Vector3i> {
        size_t operator()(const Eigen::Vector3i& v) const {
            // Combine the individual components of Eigen::Vector3i into a single hash
            size_t h1 = std::hash<int>{}(v(0));
            size_t h2 = std::hash<int>{}(v(1));
            size_t h3 = std::hash<int>{}(v(2));
            // Combine the hashes using XOR
            return h1 ^ (h2 << 1) ^ (h3 << 2); // You can use different combination methods
        }
    };
}


struct AccumulatedPoint {
    // Fields for sum of point coordinates and intensity from the first version
    Eigen::Vector3f sum_point = Eigen::Vector3f::Zero();
    float sum_intensity = 0.0f;
    int count = 0;

    // Fields for accumulating normals and colors from the second version
    int num_of_points_ = 0;
    Eigen::Vector3d normal_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d color_ = Eigen::Vector3d::Zero();

    // Add a point to the accumulated data (from both versions)
    void AddPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int idx) {
        // Accumulate the point position and intensity
        const auto& pt = cloud->points[idx];
        sum_point += Eigen::Vector3f(pt.x, pt.y, pt.z);
        sum_intensity += pt.intensity;
        count++;



        // Increment the number of points processed for normals and color
        num_of_points_++;
    }

    // Get the average point of this voxel
    pcl::PointXYZI GetAveragePoint() const {
        pcl::PointXYZI avg_pt;
        avg_pt.x = sum_point.x() / count;
        avg_pt.y = sum_point.y() / count;
        avg_pt.z = sum_point.z() / count;
        avg_pt.intensity = sum_intensity / count;
        return avg_pt;
    }

    // Get the average normal of this voxel
    Eigen::Vector3d GetAverageNormal() const {
        // Return normalized normal
        return normal_ / double(num_of_points_);
    }

    // Get the average color of this voxel
    Eigen::Vector3d GetAverageColor() const {
        return color_ / double(num_of_points_);
    }
};

inline pcl::PointCloud<pcl::PointXYZI>::Ptr VoxelDownSample(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double voxel_size) {
    
    // Check for valid voxel size
    if (voxel_size <= 0.0) {
        PCL_ERROR("[VoxelDownSample] voxel_size <= 0.\n");
        return nullptr;
    }

    // Create output cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());

    // Create variables for storing the min and max bounds
    Eigen::Vector4f min_bound, max_bound;

    // Use pcl::getMinMax3D to get the min and max bounds
    pcl::getMinMax3D(*cloud, min_bound, max_bound);  // Dereference the shared pointer

    // Calculate the min and max bounds adjusted by half the voxel size
    Eigen::Vector3f voxel_min_bound = min_bound.head<3>() - Eigen::Vector3f(voxel_size / 2, voxel_size / 2, voxel_size / 2);
    Eigen::Vector3f voxel_max_bound = max_bound.head<3>() + Eigen::Vector3f(voxel_size / 2, voxel_size / 2, voxel_size / 2);

    // Ensure that the voxel size is not too small
    if (voxel_size * std::numeric_limits<int>::max() < (voxel_max_bound - voxel_min_bound).maxCoeff()) {
        PCL_ERROR("[VoxelDownSample] voxel_size is too small.\n");
        return nullptr;
    }

    // Hash map to store voxel indices and corresponding accumulated points
    std::unordered_map<Eigen::Vector3i, AccumulatedPoint> voxel_index_to_accumulated_point;

    // Iterate through all points and assign them to voxel indices
    Eigen::Vector3f ref_coord;
    Eigen::Vector3i voxel_index;
    for (size_t i = 0; i < cloud->size(); ++i) {
        const pcl::PointXYZI& pt = cloud->points[i];
        ref_coord = (Eigen::Vector3f(pt.x, pt.y, pt.z) - voxel_min_bound) / voxel_size;
        voxel_index = Eigen::Vector3i(std::floor(ref_coord(0)), std::floor(ref_coord(1)), std::floor(ref_coord(2)));
        voxel_index_to_accumulated_point[voxel_index].AddPoint(cloud, i);
    }
    
    // Collect the average points for each voxel
    for (const auto& voxel : voxel_index_to_accumulated_point) {
        output->points.push_back(voxel.second.GetAveragePoint());
    }

    PCL_DEBUG("Pointcloud down sampled from %zu points to %zu points.\n", cloud->size(), output->size());
    return output;
}

inline cv::Mat getBEVImageParallelAndSave(
    PosePcd& pose_pcd, 
    int num_threads,
    const std::string& save_path = "",
    bool ground_view = true,
    double bev_range = 5.0,
    double bev_resolution = 0.05,
    bool enable_downsample = true,
    double voxel_size = 0.1,
    bool save_png = false,
    bool z_max_filter_enable = false,
    double z_max = 2.0)
{
    const int frameidx = pose_pcd.idx_;

    // Indoor BEV covers the closed interval [-range, range] on both axes.
    // With range=5 m and resolution=0.05 m/pixel this produces 201 x 201;
    // the downstream image preprocessing may resize it to the model input.
    const float range = static_cast<float>(bev_range);
    const float res = static_cast<float>(bev_resolution);
    if (range <= 0.0f || res <= 0.0f)
    {
        ROS_ERROR("[BEV] range and resolution must be positive (range=%.3f, resolution=%.3f)",
                  range, res);
        return cv::Mat();
    }
    const float x_min = -range, y_min = -range;
    const float x_max =  range, y_max =  range;
    const int x_min_ind = static_cast<int>(std::floor(x_min / res));
    const int x_max_ind = static_cast<int>(std::floor(x_max / res));
    const int y_min_ind = static_cast<int>(std::floor(y_min / res));
    const int y_max_ind = static_cast<int>(std::floor(y_max / res));
    const int x_num = x_max_ind - x_min_ind + 1;
    const int y_num = y_max_ind - y_min_ind + 1;

    cv::Mat mat_global_image = cv::Mat::zeros(y_num, x_num, CV_8UC1);


    if (ground_view)
    {
        // Work on a temporary copy so ceiling removal never modifies the
        // keyframe cloud later used by registration or map export.
        pcl::PointCloud<pcl::PointXYZI>::Ptr bev_input_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        if (z_max_filter_enable)
        {
            bev_input_cloud->reserve(pose_pcd.pcd_.size());
            for (const auto &point : pose_pcd.pcd_)
            {
                if (point.z <= z_max)
                    bev_input_cloud->push_back(point);
            }
        }
        else
        {
            *bev_input_cloud = pose_pcd.pcd_;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud =
            bev_input_cloud;
        if (enable_downsample)
        {
            downsampled_cloud = VoxelDownSample(bev_input_cloud, voxel_size);
        }
        if (!downsampled_cloud)
        {
            downsampled_cloud = bev_input_cloud;
        }

        // 计数缓冲：[row=y_ind, col=x_ind]
        cv::Mat count_mat = cv::Mat::zeros(y_num, x_num, CV_32SC1);

        // ===== 方位校正开关（按需改动）=====
        const bool swap_xy = true;  // 是否交换 x,y
        const bool flip_x  = true;   // 是否对 x 取反（本例：true）
        const bool flip_y  = true;   // 是否对 y 取反（本例：true）
        
        // 并行累计（先原子 +1，之后统一截断到 10）
        omp_set_num_threads(num_threads);
        #pragma omp parallel for
        for (int i = 0; i < static_cast<int>(downsampled_cloud->size()); ++i) {
            const pcl::PointXYZI& p = downsampled_cloud->at(i);

            // 轴变换
            float tx = p.x, ty = p.y;
            if (swap_xy) std::swap(tx, ty);
            if (flip_x)  tx = -tx;
            if (flip_y)  ty = -ty;

            // Preserve the original closed-boundary projection and orientation.
            const int x_ind = x_max_ind - static_cast<int>(std::floor(ty / res));
            const int y_ind = y_max_ind - static_cast<int>(std::floor(tx / res));
            if (x_ind < 0 || y_ind < 0 || x_ind >= x_num || y_ind >= y_num) continue;

            int& cell = count_mat.at<int>(y_ind, x_ind);
            #pragma omp atomic
            cell += 1;
        }

        // 统一截断到 10。
        count_mat.setTo(10, (count_mat > 10));

        // 强度映射：固定步长 50，不归一化（与 getBEVv2 对齐）
        for (int r = 0; r < y_num; ++r) {
            const int* cr = count_mat.ptr<int>(r);
            uchar*     gr = mat_global_image.ptr<uchar>(r);
            for (int c = 0; c < x_num; ++c) {
                int val = cr[c] * 50;
                if (val > 255) val = 255;
                gr[c] = static_cast<uchar>(val);
            }
        }
    }
    else
    {
        // ========= 你原来高度视角分支，修正索引顺序为 [row=y][col=x] =========
        pcl::PointCloud<pcl::PointXYZI>::Ptr bev_input_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        if (z_max_filter_enable)
        {
            bev_input_cloud->reserve(pose_pcd.acc_pcd.size());
            for (const auto &point : pose_pcd.acc_pcd)
            {
                if (point.z <= z_max)
                    bev_input_cloud->push_back(point);
            }
        }
        else
        {
            *bev_input_cloud = pose_pcd.acc_pcd;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud =
            bev_input_cloud;
        if (enable_downsample)
        {
            downsampled_cloud = VoxelDownSample(bev_input_cloud, voxel_size);
        }
        if (!downsampled_cloud)
        {
            downsampled_cloud = bev_input_cloud;
        }
        
        cv::Mat height_sum  = cv::Mat::zeros(y_num, x_num, CV_32FC1);
        cv::Mat point_count = cv::Mat::zeros(y_num, x_num, CV_32SC1);

        // 找最小 z'（z' = -x）
        float min_tz = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) {
            float tz = -downsampled_cloud->at(i).x;  // z' = -x
            if (tz < min_tz) min_tz = tz;
        }
        
        // 累计高度和计数：x' = z, y' = y, z' = -x
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) {
            const pcl::PointXYZI& p = downsampled_cloud->at(i);
            float tx = p.z;   // x'
            float ty = p.y;   // y'
            float tz = -p.x;  // z'

            const int x_ind = x_max_ind - static_cast<int>(std::floor(tx / res));
            const int y_ind = y_max_ind - static_cast<int>(std::floor(ty / res));
            if (x_ind < 0 || y_ind < 0 || x_ind >= x_num || y_ind >= y_num) continue;

            float h = std::fabs(tz - min_tz);
            height_sum.at<float>(y_ind, x_ind)  += h;   // (row=y, col=x)
            point_count.at<int>(y_ind, x_ind)   += 1;
        }

        // 平均高度
        cv::Mat bev_height = cv::Mat::zeros(y_num, x_num, CV_32FC1);
        for (int r = 0; r < y_num; ++r) {
            const float* hsum_ptr = height_sum.ptr<float>(r);
            const int*   cnt_ptr  = point_count.ptr<int>(r);
            float*       out_ptr  = bev_height.ptr<float>(r);
            for (int c = 0; c < x_num; ++c) {
                if (cnt_ptr[c] > 0) {
                    out_ptr[c] = hsum_ptr[c] / static_cast<float>(cnt_ptr[c]);
                }
            }
        }

        // 归一化到 0-255（上限裁到 20m）
        double min_val, max_val;
        cv::minMaxLoc(bev_height, &min_val, &max_val);
        const float upper = static_cast<float>(std::min(max_val, 20.0));
        mat_global_image = cv::Mat::zeros(y_num, x_num, CV_8UC1);

        if (upper > 0.0f) {
            const float scale = 255.0f / upper;
            for (int r = 0; r < y_num; ++r) {
                const float* src = bev_height.ptr<float>(r);
                uchar* dst = mat_global_image.ptr<uchar>(r);
                for (int c = 0; c < x_num; ++c) {
                    float val = src[c] * scale;
                    dst[c] = cv::saturate_cast<uchar>(val);
                }
            }
        } else {
            std::cout << "[Warning] BEV upper height is " << upper << std::endl;
        }
    }

    // 可选保存图像（默认关闭，避免关键帧路径被磁盘 I/O 拖慢）
    if (save_png && !save_path.empty()) {
        const std::string save_file = save_path + "/" + std::to_string(frameidx) + ".png";
        cv::imwrite(save_file, mat_global_image);
    }

    return mat_global_image;
}
