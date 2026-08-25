// #include <so3_math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64MultiArray.h>
#include "li_initialization.h"
#include "map_backend/ivox_backend.h"
#include "map_backend/octvox_backend.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <limits>
#include <malloc.h>
#include <memory>
// #include <cv_bridge/cv_bridge.h>
// #include "matplotlibcpp.h"
// #include <ros/console.h>

using namespace std;     

#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

namespace
{
std::shared_ptr<LocalMapBackend> createLocalMapBackend()
{
    if (local_map_backend_name == "octvox")
    {
        ROS_INFO("[PointLIOMap] backend=octvox resolution=%.3f capacity=%d k=5",
                 octvox_resolution, octvox_capacity);
        return std::make_shared<OctVoxBackend>(
            octvox_resolution, static_cast<std::size_t>(octvox_capacity));
    }
    if (local_map_backend_name != "ivox")
    {
        ROS_WARN("[PointLIOMap] unknown backend '%s', fallback to ivox",
                 local_map_backend_name.c_str());
    }
    ROS_INFO("[PointLIOMap] backend=ivox resolution=%.3f k=5",
             ivox_options_.resolution_);
    return std::make_shared<IVoxBackend<IVoxType>>(ivox_options_);
}
}  // namespace

string root_dir = ROOT_DIR;

int time_log_counter = 0; //, publish_count = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool  flg_reset = false, flg_exit = false;
std::atomic<bool> reset_local_map_requested(false);

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;

struct TimedLidarPose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double offset_ms = 0.0;
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    V3D translation = V3D::Zero();
};

using TimedLidarPoseVector =
    std::vector<TimedLidarPose, Eigen::aligned_allocator<TimedLidarPose>>;

TimedLidarPoseVector scan_lidar_poses;

void get_current_lidar_pose(Eigen::Quaterniond &rotation, V3D &translation)
{
    M3D rotation_world_imu;
    M3D rotation_imu_lidar;
    V3D translation_world_imu;
    V3D translation_imu_lidar;

    if (!use_imu_as_input)
    {
        rotation_world_imu = kf_output.x_.rot;
        translation_world_imu = kf_output.x_.pos;
        rotation_imu_lidar = extrinsic_est_en ?
            M3D(kf_output.x_.offset_R_L_I) : Lidar_R_wrt_IMU;
        translation_imu_lidar = extrinsic_est_en ?
            V3D(kf_output.x_.offset_T_L_I) : Lidar_T_wrt_IMU;
    }
    else
    {
        rotation_world_imu = kf_input.x_.rot;
        translation_world_imu = kf_input.x_.pos;
        rotation_imu_lidar = extrinsic_est_en ?
            M3D(kf_input.x_.offset_R_L_I) : Lidar_R_wrt_IMU;
        translation_imu_lidar = extrinsic_est_en ?
            V3D(kf_input.x_.offset_T_L_I) : Lidar_T_wrt_IMU;
    }

    rotation = Eigen::Quaterniond(rotation_world_imu * rotation_imu_lidar).normalized();
    translation = rotation_world_imu * translation_imu_lidar + translation_world_imu;
}

void record_current_lidar_pose(double offset_ms)
{
    TimedLidarPose pose;
    pose.offset_ms = offset_ms;
    get_current_lidar_pose(pose.rotation, pose.translation);

    auto position = std::lower_bound(
        scan_lidar_poses.begin(), scan_lidar_poses.end(), offset_ms,
        [](const TimedLidarPose &sample, double timestamp_ms)
        {
            return sample.offset_ms < timestamp_ms;
        });
    if (position != scan_lidar_poses.end() &&
        std::abs(position->offset_ms - offset_ms) < 1e-6)
    {
        *position = pose;
        return;
    }

    scan_lidar_poses.insert(position, pose);
}

PointCloudXYZI::Ptr deskew_full_resolution_scan_to_world()
{
    PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI(feats_undistort->size(), 1));
    if (feats_undistort->empty())
    {
        return cloud_world;
    }

    if (scan_lidar_poses.empty())
    {
        ROS_WARN_THROTTLE(5.0,
            "No Point-LIO pose samples for full-resolution deskew; using the current pose.");
        record_current_lidar_pose(feats_undistort->points.back().curvature);
    }

    for (size_t i = 0; i < feats_undistort->size(); ++i)
    {
        const PointType &point_lidar = feats_undistort->points[i];
        const double offset_ms = point_lidar.curvature;

        auto upper = std::lower_bound(
            scan_lidar_poses.begin(), scan_lidar_poses.end(), offset_ms,
            [](const TimedLidarPose &pose, double timestamp_ms)
            {
                return pose.offset_ms < timestamp_ms;
            });

        Eigen::Quaterniond rotation;
        V3D translation;
        if (upper == scan_lidar_poses.begin())
        {
            rotation = upper->rotation;
            translation = upper->translation;
        }
        else if (upper == scan_lidar_poses.end())
        {
            rotation = scan_lidar_poses.back().rotation;
            translation = scan_lidar_poses.back().translation;
        }
        else
        {
            const TimedLidarPose &pose_after = *upper;
            const TimedLidarPose &pose_before = *(upper - 1);
            const double duration_ms = pose_after.offset_ms - pose_before.offset_ms;
            const double ratio = duration_ms > 1e-9 ?
                (offset_ms - pose_before.offset_ms) / duration_ms : 0.0;
            rotation = pose_before.rotation.slerp(ratio, pose_after.rotation).normalized();
            translation = (1.0 - ratio) * pose_before.translation + ratio * pose_after.translation;
        }

        const V3D point_world =
            rotation * V3D(point_lidar.x, point_lidar.y, point_lidar.z) + translation;
        PointType &output = cloud_world->points[i];
        output = point_lidar;
        output.x = point_world.x();
        output.y = point_world.y();
        output.z = point_world.z();
    }

    return cloud_world;
}

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

bool resetLocalMapService(std_srvs::Trigger::Request &,
                          std_srvs::Trigger::Response &response)
{
    reset_local_map_requested.store(true);
    response.success = true;
    response.message = "PointLIO local map reset scheduled; ESKF/IMU state is preserved";
    return true;
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang;
    if (!use_imu_as_input)
    {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    }
    else
    {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }
    
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    if (use_imu_as_input)
    {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a  
    }
    else
    {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1), kf_output.x_.gravity(2)); // Bias_a  
    }
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en)
    {
        if (!use_imu_as_input)
        {
            p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
        }
        else
        {
            p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    }
    else
    {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void MapIncremental() {
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);

    for (size_t i = 0; i < cur_pts; ++i) {
        /* decide if need add to map */
        PointType &point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
            bool need_add = true;
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    }
    {
        const auto insert_begin = std::chrono::steady_clock::now();
        local_map_backend_->addPoints(points_to_add);
        map_backend_stats.insert_total_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - insert_begin)
                .count();
    }
}

void publish_init_map(const ros::Publisher & pubLaserCloudFullRes)
{
    int size_init_map = init_feats_world->size();

    sensor_msgs::PointCloud2 laserCloudmsg;
                
    pcl::toROSMsg(*init_feats_world, laserCloudmsg);
        
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = publish_world_frame;
    pubLaserCloudFullRes.publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes,
                         const ros::Publisher & pubLaserCloudDownsampled)
{
    if (scan_world_publish_en || scan_world_downsample_publish_en)
    {
        PointCloudXYZI::Ptr laserCloudWorld = deskew_full_resolution_scan_to_world();
        sensor_msgs::PointCloud2 full_resolution_msg;
        pcl::toROSMsg(*laserCloudWorld, full_resolution_msg);
        full_resolution_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
        full_resolution_msg.header.frame_id = publish_world_frame;
        if (scan_world_publish_en)
            pubLaserCloudFullRes.publish(full_resolution_msg);

        sensor_msgs::PointCloud2 downsampled_msg;
        pcl::toROSMsg(*feats_down_world, downsampled_msg);
        downsampled_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
        downsampled_msg.header.frame_id = publish_world_frame;
        if (scan_world_downsample_publish_en)
            pubLaserCloudDownsampled.publish(downsampled_msg);
        // publish_count -= PUBFRAME_PERIOD;
    }
    
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    // publish_count -= PUBFRAME_PERIOD;
}

template<typename T>
void set_posestamp(T & out)
{
    if (!use_imu_as_input)
    {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        Eigen::Quaterniond q(kf_output.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
    else
    {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        Eigen::Quaterniond q(kf_input.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
}

void set_twiststamp(geometry_msgs::Twist &out)
{
    V3D linear_velocity_body;
    V3D angular_velocity_body;

    if (!use_imu_as_input)
    {
        // The filter velocity is expressed in the odometry/world frame, while
        // nav_msgs/Odometry requires twist in child_frame_id ("body").
        linear_velocity_body =
            M3D(kf_output.x_.rot).transpose() * kf_output.x_.vel;
        // omg is estimated in the IMU/body frame in the output-state model.
        angular_velocity_body = kf_output.x_.omg;
    }
    else
    {
        linear_velocity_body =
            M3D(kf_input.x_.rot).transpose() * kf_input.x_.vel;
        // In the input-state model angular velocity is the bias-corrected gyro
        // measurement, also expressed in the IMU/body frame.
        input_in.gyro.boxminus(angular_velocity_body, kf_input.x_.bg);
    }

    out.linear.x = linear_velocity_body.x();
    out.linear.y = linear_velocity_body.y();
    out.linear.z = linear_velocity_body.z();
    out.angular.x = angular_velocity_body.x();
    out.angular.y = angular_velocity_body.y();
    out.angular.z = angular_velocity_body.z();
}

double sensorAgeSeconds(double sensor_stamp)
{
    const double ros_now = ros::Time::now().toSec();
    if (sensor_stamp <= 0.0 || ros_now <= 0.0 || ros_now < sensor_stamp)
        return -1.0;
    return ros_now - sensor_stamp;
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = publish_world_frame;
    odomAftMapped.child_frame_id = "body";
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(time_current);
    }
    else
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);
    set_twiststamp(odomAftMapped.twist.twist);

    // Limit the original PointLIO odometry topic itself. High-rate mode still
    // computes scan-internal states, but only the newest state due at this
    // sensor timestamp is published. A negative limit explicitly disables
    // throttling and preserves the upstream unlimited behavior.
    static double last_publish_stamp = -1.0;
    const double current_stamp = odomAftMapped.header.stamp.toSec();

    // Diagnostic-only sanity guard. It never rewrites the pose; it only logs
    // the first frame of a divergent run together with queue/queue-span state.
    {
        static double last_guard_stamp = -1.0;
        static double last_guard_x = 0.0;
        static double last_guard_y = 0.0;
        static double last_guard_yaw = 0.0;
        static bool guard_initialized = false;

        const geometry_msgs::Point &pos = odomAftMapped.pose.pose.position;
        const geometry_msgs::Quaternion &quat =
            odomAftMapped.pose.pose.orientation;
        const geometry_msgs::Vector3 &lin = odomAftMapped.twist.twist.linear;
        const geometry_msgs::Vector3 &ang = odomAftMapped.twist.twist.angular;
        const bool finite_ok =
            std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z) &&
            std::isfinite(quat.x) && std::isfinite(quat.y) &&
            std::isfinite(quat.z) && std::isfinite(quat.w) &&
            std::isfinite(lin.x) && std::isfinite(lin.y) && std::isfinite(lin.z) &&
            std::isfinite(ang.x) && std::isfinite(ang.y) && std::isfinite(ang.z);

        const double yaw = atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                                 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
        double guard_dt = 0.0;
        double dtrans = 0.0;
        double dyaw_deg = 0.0;
        if (guard_initialized && current_stamp > last_guard_stamp)
        {
            guard_dt = current_stamp - last_guard_stamp;
            const double dx = pos.x - last_guard_x;
            const double dy = pos.y - last_guard_y;
            dtrans = std::sqrt(dx * dx + dy * dy);
            dyaw_deg = std::fabs(yaw - last_guard_yaw) * 180.0 / M_PI;
            if (dyaw_deg > 180.0)
                dyaw_deg = 360.0 - dyaw_deg;
        }

        if (!finite_ok || dtrans > 2.0 || dyaw_deg > 45.0)
        {
            const SensorBufferStats stats = getSensorBufferStats();
            const double span =
                (stats.lidar_back_stamp >= stats.lidar_front_stamp &&
                 stats.lidar_front_stamp >= 0.0)
                    ? (stats.lidar_back_stamp - stats.lidar_front_stamp) : 0.0;
            ROS_WARN_THROTTLE(1.0,
                "[PointLIO StateGuard] stamp=%.6f dt=%.4f dtrans=%.3f "
                "dyaw_deg=%.2f pos=(%.3f,%.3f,%.3f) processing_age=%.3f "
                "lidar_q=%zu imu_q=%zu queue_span=%.3f",
                current_stamp, guard_dt, dtrans, dyaw_deg,
                pos.x, pos.y, pos.z,
                sensorAgeSeconds(lidar_end_time),
                stats.lidar_queue_size, stats.imu_queue_size, span);
        }

        last_guard_stamp = current_stamp;
        last_guard_x = pos.x;
        last_guard_y = pos.y;
        last_guard_yaw = yaw;
        guard_initialized = true;
    }

    if (current_stamp < last_publish_stamp)
        last_publish_stamp = -1.0;
    if (odometry_max_publish_hz == 0.0)
        return;
    if (odometry_max_publish_hz > 0.0 && last_publish_stamp >= 0.0 &&
        current_stamp - last_publish_stamp + 1.0e-9 <
            1.0 / odometry_max_publish_hz)
        return;

    pubOdomAftMapped.publish(odomAftMapped);
    last_publish_stamp = current_stamp;

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    const tf::Vector3 body_position(
        odomAftMapped.pose.pose.position.x,
        odomAftMapped.pose.pose.position.y,
        odomAftMapped.pose.pose.position.z);
    const tf::Vector3 child_to_body(
        odometry_tf_child_to_body_x_m,
        odometry_tf_child_to_body_y_m,
        odometry_tf_child_to_body_z_m);
    // PointLIO estimates the body/MID360 origin. When the configured TF child
    // is the vehicle center, shift the published pose backwards by the rigid
    // child->body offset while retaining the original body odometry message.
    transform.setOrigin(body_position - tf::quatRotate(q, child_to_body));
    transform.setRotation( q );
    br.sendTransform(tf::StampedTransform(
        transform, odomAftMapped.header.stamp, publish_world_frame,
        odometry_tf_child_frame));
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = publish_world_frame;
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath.publish(path);
    }
}        

void reportNegativeDt(const char* where, double dt, double t_cur, double t_last)
{
    const SensorBufferStats stats = getSensorBufferStats();
    ROS_ERROR_THROTTLE(1.0,
        "[PointLIO DT] %s negative dt=%.9f t_cur=%.6f t_last=%.6f "
        "lidar_beg=%.6f lidar_end=%.6f lidar_q=%zu imu_q=%zu span=%.3f",
        where, dt, t_cur, t_last, Measures.lidar_beg_time, lidar_end_time,
        stats.lidar_queue_size, stats.imu_queue_size,
        (stats.lidar_back_stamp >= stats.lidar_front_stamp &&
         stats.lidar_front_stamp >= 0.0)
            ? (stats.lidar_back_stamp - stats.lidar_front_stamp) : 0.0);
}

// Fixed Float64MultiArray layout, indices documented here:
// 0 wall_now_s, 1 latest_lidar_rx_stamp_s, 2 latest_imu_rx_stamp_s,
// 3 processing_lidar_begin_stamp_s, 4 processing_lidar_end_stamp_s,
// 5 processing_age_s, 6 lidar_front_age_s (-1 when empty),
// 7 lidar_queue_span_s, 8 lidar_queue_size, 9 imu_queue_size,
// 10 scan_compute_ms, 11 cloud_publish_ms, 12 path_pose_count,
// 13 dropped_lidar_scans_total.
void publish_runtime_metrics(const ros::Publisher &pub, double scan_ms,
                             double cloud_ms, double match_ms, double solve_ms,
                             double propag_ms, double update_ms, double icp_ms,
                             double map_ms)
{
    const double wall_now = ros::WallTime::now().toSec();
    const SensorBufferStats stats = getSensorBufferStats();
    std_msgs::Float64MultiArray msg;
    msg.data.resize(21, 0.0);
    msg.data[0] = wall_now;
    msg.data[1] = stats.latest_lidar_stamp;
    msg.data[2] = stats.latest_imu_stamp;
    msg.data[3] = Measures.lidar_beg_time;
    msg.data[4] = lidar_end_time;
    msg.data[5] = sensorAgeSeconds(lidar_end_time);
    msg.data[6] = stats.lidar_front_stamp >= 0.0
                      ? sensorAgeSeconds(stats.lidar_front_stamp) : -1.0;
    msg.data[7] = (stats.lidar_back_stamp >= stats.lidar_front_stamp &&
                   stats.lidar_front_stamp >= 0.0)
                      ? (stats.lidar_back_stamp - stats.lidar_front_stamp) : 0.0;
    msg.data[8] = static_cast<double>(stats.lidar_queue_size);
    msg.data[9] = static_cast<double>(stats.imu_queue_size);
    msg.data[10] = scan_ms;
    msg.data[11] = cloud_ms;
    msg.data[12] = static_cast<double>(path.poses.size());
    msg.data[13] = static_cast<double>(
        dropped_lidar_scans_total.load(std::memory_order_relaxed));
    msg.data[14] = static_cast<double>(map_backend_stats.knn_queries);
    msg.data[15] = static_cast<double>(map_backend_stats.knn_success);
    msg.data[16] = map_backend_stats.knn_queries > 0
                       ? map_backend_stats.knn_total_ms * 1.0e3 /
                             static_cast<double>(map_backend_stats.knn_queries)
                       : 0.0;  // knn avg us
    msg.data[17] = map_backend_stats.knn_max_ms * 1.0e3;  // knn max us
    msg.data[18] = map_backend_stats.insert_total_ms;
    msg.data[19] = static_cast<double>(
        local_map_backend_ ? local_map_backend_->numCells() : 0);
    msg.data[20] = static_cast<double>(
        local_map_backend_ ? local_map_backend_->numRepresentatives() : 0);
    pub.publish(msg);

    ROS_INFO_THROTTLE(
        1.0,
        "[PointLIOMapStats] backend=%s queries=%lu success=%.3f "
        "knn_avg_us=%.1f knn_max_us=%.1f insert_ms=%.2f cells=%lu reps=%lu",
        local_map_backend_ ? local_map_backend_->name() : "none",
        static_cast<unsigned long>(map_backend_stats.knn_queries),
        map_backend_stats.knn_queries > 0
            ? static_cast<double>(map_backend_stats.knn_success) /
                  static_cast<double>(map_backend_stats.knn_queries)
            : 0.0,
        msg.data[16], msg.data[17], msg.data[18],
        static_cast<unsigned long>(msg.data[19]),
        static_cast<unsigned long>(msg.data[20]));

    map_backend_stats.knn_queries = 0;
    map_backend_stats.knn_success = 0;
    map_backend_stats.knn_total_ms = 0.0;
    map_backend_stats.knn_max_ms = 0.0;
    map_backend_stats.insert_total_ms = 0.0;

    const double processing_age = msg.data[5];
    if (processing_age > 0.15)
    {
        ROS_WARN_THROTTLE(1.0,
            "[PointLIO RT] age=%.3f lidar_q=%zu imu_q=%zu span=%.3f "
            "scan=%.1fms match=%.1f solve=%.1f prop=%.1f upd=%.1f "
            "icp=%.1f map=%.1f cloud=%.1fms dropped=%lu",
            processing_age, stats.lidar_queue_size, stats.imu_queue_size,
            msg.data[7], scan_ms, match_ms, solve_ms, propag_ms, update_ms,
            icp_ms, map_ms, cloud_ms,
            dropped_lidar_scans_total.load(std::memory_order_relaxed));
    }
    else
    {
        ROS_INFO_THROTTLE(1.0,
            "[PointLIO RT] age=%.3f lidar_q=%zu imu_q=%zu span=%.3f "
            "scan=%.1fms match=%.1f solve=%.1f prop=%.1f upd=%.1f "
            "icp=%.1f map=%.1f cloud=%.1fms dropped=%lu",
            processing_age, stats.lidar_queue_size, stats.imu_queue_size,
            msg.data[7], scan_ms, match_ms, solve_ms, propag_ms, update_ms,
            icp_ms, map_ms, cloud_ms,
            dropped_lidar_scans_total.load(std::memory_order_relaxed));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh("~");

    // Parameters must be loaded before selecting the callback execution
    // model. In particular, avoid AsyncSpinner(0): on the N100 it can consume
    // every core and starve the estimator during a backend latency spike.
    readParameters(nh);

    // Phase-0 A/B switch. With AsyncSpinner several sensor callbacks may run
    // concurrently; the single-thread mode serialises callbacks on the main
    // thread and is only meant to test whether the random divergence is
    // concurrency related.
    bool single_thread_sensor_callbacks = false;
    nh.param<bool>("debug/single_thread_sensor_callbacks",
                   single_thread_sensor_callbacks, false);
    std::unique_ptr<ros::AsyncSpinner> spinner;
    if (!single_thread_sensor_callbacks)
    {
        spinner.reset(new ros::AsyncSpinner(sensor_callback_threads));
        spinner->start();
    }

    cout<<"lidar_type: "<<lidar_type<<endl;
    local_map_backend_ = createLocalMapBackend();
    
    path.header.stamp    = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = publish_world_frame;

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    
        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    
    if (extrinsic_est_en)
    {
        if (!use_imu_as_input)
        {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
        else
        {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 24, 24> P_init; // = MD(18, 18)::Identity() * 0.1;
    reset_cov(P_init);
    kf_input.change_P(P_init);
    Eigen::Matrix<double, 30, 30> P_init_output; // = MD(24, 24)::Identity() * 0.01;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");
    open_file();

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, lidar_sub_queue_size, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, lidar_sub_queue_size, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, imu_sub_queue_size, imu_cbk);
    ros::ServiceServer reset_local_map_service =
        nh.advertiseService("/pointlio/reset_local_map", resetLocalMapService);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", cloud_pub_queue_size);
    ros::Publisher pubLaserCloudDownsampled = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_ds", cloud_pub_queue_size);
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", cloud_pub_queue_size);
    recovery_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
            ("/pointlio/recovery_cloud_body_raw", cloud_pub_queue_size);
    // ros::Publisher pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>
            // ("/cloud_effected", 1000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 1000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/aft_mapped_to_init", odom_pub_queue_size);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", path_pub_queue_size);
    ros::Publisher runtime_metrics_pub =
        nh.advertise<std_msgs::Float64MultiArray>("/pointlio/runtime_metrics", 10);
    // ros::Publisher plane_pub = nh.advertise<visualization_msgs::Marker>
            // ("/planner_normal", 1000);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate loop_rate(500);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        if (single_thread_sensor_callbacks)
            ros::spinOnce();
        if (reset_local_map_requested.exchange(false))
        {
            init_map = false;
            init_feats_world.reset(new PointCloudXYZI());
            depth_feats_world.clear();
            local_map_backend_->clear();
            ROS_WARN("PointLIO local map reset by localization backend; ESKF/IMU state preserved.");
        }
        if(sync_packages(Measures)) 
        {
            if (flg_reset)
            {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                feats_undistort.reset(new PointCloudXYZI());
                if (use_imu_as_input)
                {
                    // state_in = kf_input.get_x();
                    state_in = state_input();
                    kf_input.change_P(P_init);
                }
                else
                {
                    // state_out = kf_output.get_x();
                    state_out = state_output();
                    kf_output.change_P(P_init_output);
                }
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                init_map = false;
                
                {
                    local_map_backend_->clear();
                }
            }

            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                if (first_imu_time < 1)
                {
                    first_imu_time = imu_next.header.stamp.toSec();
                    printf("first imu time: %f\n", first_imu_time);
                }
                time_current = 0.0;
                if(imu_en)
                {
                    // imu_next = *(imu_deque.front());
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc *= -1; 

                    {
                        sensor_msgs::Imu next;
                        while (Measures.lidar_beg_time > imu_next.header.stamp.toSec()) // if it is needed for the new map?
                        {
                            if (!popImuFrontAndGetNext(next))
                                break;
                            imu_last = imu_next;
                            imu_next = next;
                        }
                    }
                }
                else
                {
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity); // _init);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc *= -1; 
                    p_imu->imu_need_init_ = false;
                    // p_imu->after_imu_init_ = true;
                }     
                G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();
            
            /*** downsample the feature points in a scan ***/
            t1 = omp_get_wtime();
            p_imu->Process(Measures, feats_undistort);
            scan_lidar_poses.clear();
            if(space_down_sample)
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            else
            {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            {
                time_seq = time_compressing<int>(feats_down_body);
                feats_down_size = feats_down_body->points.size();
            }

            if (!p_imu->after_imu_init_) // !p_imu->UseLIInit && 
            {
                if (!p_imu->imu_need_init_)
                { 
                    V3D tmp_gravity;
                    if (imu_en)
                    {tmp_gravity = - p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;}
                    else
                    {tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                    p_imu->after_imu_init_ = true;
                    }
                    // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    kf_input.x_.rot = rot_init;
                    kf_output.x_.rot = rot_init;
                    // kf_input.x_.rot; //.normalize();
                    // kf_output.x_.rot; //.normalize();
                    kf_output.x_.acc = - rot_init.transpose() * kf_output.x_.gravity;
                }
                else{
                continue;}
            }
            /*** initialize the map ***/
            if(!init_map)
            {
                feats_down_world->resize(feats_undistort->size());
                for(int i = 0; i < feats_undistort->size(); i++)
                {
                    {
                        pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
                    }
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) 
                {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if(init_feats_world->size() < init_map_size) 
                {init_map = false;}
                else
                {   
                    local_map_backend_->addPoints(init_feats_world->points);
                    publish_init_map(pubLaserCloudMap); //(pubLaserCloudFullRes);
                    
                    init_feats_world.reset(new PointCloudXYZI());
                    init_map = true;
                }
                continue;
            }

            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            // The current state is the best available pose at the beginning of
            // this scan. Point-wise filter updates below add the rest of the
            // trajectory used to deskew the untouched, full-resolution scan.
            record_current_lidar_pose(0.0);

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            // reserve() only grows capacity; operator[] needs a real element.
            // Use resize() so the point-wise writes below stay in bounds.
            crossmat_list.resize(feats_down_size);
            pbody_list.resize(feats_down_size);
            assert(pbody_list.size() >= feats_down_body->size());
            if (!extrinsic_est_en)
            {
                assert(crossmat_list.size() >= feats_down_body->size());
            }
                          
            for (size_t i = 0; i < feats_down_body->size(); i++)
            {
                V3D point_this(feats_down_body->points[i].x,
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;
                if (!extrinsic_est_en)
                // {
                //     if (!use_imu_as_input)
                //     {
                //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
                //     }
                //     else
                //     {
                //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
                //     }
                // }
                // else
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);
                    crossmat_list[i]=point_crossmat;
                }
            }
            if (!use_imu_as_input)
            {     
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /**** point by point update ****/
                if (time_seq.size() > 0)
                {
                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];

                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame)
                    {
                        if(imu_en)
                        {
                            sensor_msgs::Imu next;
                            while (time_current > imu_next.header.stamp.toSec())
                            {
                                if (!popImuFrontAndGetNext(next)) break;
                                imu_last = imu_next;
                                imu_next = next;
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                        }
                        is_first_frame = false;
                        imu_upda_cov = true;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if(imu_en)
                    {
                        sensor_msgs::Imu front;
                        if (imuBufferFront(front))
                        {
                            bool last_imu = imu_next.header.stamp.toSec() == front.header.stamp.toSec();
                            while (imu_next.header.stamp.toSec() < time_predict_last_const)
                            {
                                if (!last_imu)
                                {
                                    imu_last = imu_next;
                                    imu_next = front;
                                    break;
                                }
                                sensor_msgs::Imu next;
                                if (!popImuFrontAndGetNext(next)) break;
                                imu_last = imu_next;
                                imu_next = next;
                            }
                            bool imu_comes = time_current > imu_next.header.stamp.toSec();
                            while (imu_comes)
                            {
                                imu_upda_cov = true;
                                angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                                acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                                /*** covariance update ***/
                                double dt = imu_next.header.stamp.toSec() - time_predict_last_const;
                                if (dt < -1e-3)
                                {
                                    reportNegativeDt("imu cov update", dt,
                                                     imu_next.header.stamp.toSec(),
                                                     time_predict_last_const);
                                    break;
                                }
                                if (dt < 0.0) dt = 0.0;
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = imu_next.header.stamp.toSec(); // big problem

                                {
                                    double dt_cov = imu_next.header.stamp.toSec() - time_update_last;

                                    if (dt_cov > 0.0)
                                    {
                                        time_update_last = imu_next.header.stamp.toSec();
                                        double propag_imu_start = omp_get_wtime();

                                        kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                        propag_time += omp_get_wtime() - propag_imu_start;
                                        double solve_imu_start = omp_get_wtime();
                                        kf_output.update_iterated_dyn_share_IMU();
                                        solve_time += omp_get_wtime() - solve_imu_start;
                                    }
                                }
                                sensor_msgs::Imu next;
                                if (!popImuFrontAndGetNext(next)) break;
                                imu_last = imu_next;
                                imu_next = next;
                                imu_comes = time_current > imu_next.header.stamp.toSec();
                            }
                        }
                    }
                    if (flg_reset)
                    {
                        break;
                    }

                    double dt = time_current - time_predict_last_const;
                    if (dt < -1e-3)
                    {
                        reportNegativeDt("point predict", dt, time_current,
                                         time_predict_last_const);
                        break;
                    }
                    if (dt < 0.0) dt = 0.0;
                    double propag_state_start = omp_get_wtime();
                    if(!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;   
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;
                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1)
                    {
                        ROS_WARN("No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    const bool lidar_update_succeeded =
                        kf_output.update_iterated_dyn_share_modified();
                    record_current_lidar_pose(point_body.curvature);
                    if (!lidar_update_succeeded)
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }
                    solve_start = omp_get_wtime();
                        
                    if (publish_odometry_without_downsample)
                    {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped);
                        if (runtime_pos_log)
                        {
                            euler_cur = SO3ToEuler(kf_output.x_.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " " << kf_output.x_.vel.transpose() \
                            <<" "<<kf_output.x_.omg.transpose()<<" "<<kf_output.x_.acc.transpose()<<" "<<kf_output.x_.gravity.transpose()<<" "<<kf_output.x_.bg.transpose()<<" "<<kf_output.x_.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                
                    solve_time += omp_get_wtime() - solve_start;
    
                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                    // cout << "pbp output effect feat num:" << effct_feat_num << endl;
                }
                }
                else
                {
                    sensor_msgs::Imu front;
                    if (imuBufferFront(front))
                    {
                        imu_last = imu_next;
                        imu_next = front;

                    while (imu_next.header.stamp.toSec() > time_current && ((imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte )))
                    { // >= ?
                        if (is_first_frame)
                        {
                            {
                                {
                                    sensor_msgs::Imu next;
                                    while (imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte)
                                    {
                                        // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                        if (!popImuFrontAndGetNext(next)) break;
                                        imu_last = imu_next;
                                        imu_next = next;
                                    }
                                }
                                break;
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                                            
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                            imu_upda_cov = true;
                            time_update_last = time_current;
                            time_predict_last_const = time_current;

                                is_first_frame = false;
                        }
                        time_current = imu_next.header.stamp.toSec();

                        if (!is_first_frame)
                        {
                        double dt = time_current - time_predict_last_const;
                        {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0)
                            {
                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                time_update_last = time_current;
                            }
                            kf_output.predict(dt, Q_output, input_in, true, false);
                        }

                        time_predict_last_const = time_current;

                        angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z; 
                        // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                        kf_output.update_iterated_dyn_share_IMU();
                        sensor_msgs::Imu next;
                        if (!popImuFrontAndGetNext(next)) break;
                        imu_last = imu_next;
                        imu_next = next;
                    }
                    else
                    {
                        sensor_msgs::Imu next;
                        if (!popImuFrontAndGetNext(next)) break;
                        imu_last = imu_next;
                        imu_next = next;
                    }
                    }
                    }
                }
            }
            else
            {
                bool imu_prop_cov = false;
                effct_feat_num = 0;
                if (time_seq.size() > 0)
                {
                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (is_first_frame)
                    {
                        sensor_msgs::Imu next;
                        while (time_current > imu_next.header.stamp.toSec()) 
                        {
                            if (!popImuFrontAndGetNext(next)) break;
                            imu_last = imu_next;
                            imu_next = next;
                        }
                        imu_prop_cov = true;

                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current; 
                        {
                            input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;                 
                            input_in.acc<<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        }
                    }
                    
                    while (time_current > imu_next.header.stamp.toSec()) // && !imu_deque.empty())
                    {
                        imuBufferPopFront();
                        
                        input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z; 
                        input_in.acc    = input_in.acc * G_m_s2 / acc_norm; 
                        double dt = imu_last.header.stamp.toSec() - t_last;
                        if (dt < -1e-3)
                        {
                            reportNegativeDt("imu input propagate", dt,
                                             imu_last.header.stamp.toSec(),
                                             t_last);
                            break;
                        }
                        if (dt < 0.0) dt = 0.0;

                        double dt_cov = imu_last.header.stamp.toSec() - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                            time_update_last = imu_last.header.stamp.toSec(); //time_current;
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false); 
                        t_last = imu_last.header.stamp.toSec();
                        imu_prop_cov = true;

                        sensor_msgs::Imu next;
                        if (!imuBufferFront(next)) break;
                        imu_last = imu_next;
                        imu_next = next;
                        // imu_upda_cov = true;
                    }     
                    if (flg_reset)
                    {
                        break;
                    }     
                    double dt = time_current - t_last;
                    if (dt < -1e-3)
                    {
                        reportNegativeDt("point predict", dt, time_current,
                                         t_last);
                        break;
                    }
                    if (dt < 0.0) dt = 0.0;
                    t_last = time_current;
                    double propag_start = omp_get_wtime();
                    
                    if(!prop_at_freq_of_imu)
                    {   
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {    
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                            time_update_last = time_current; 
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false); 

                    propag_time += omp_get_wtime() - propag_start;

                    double t_update_start = omp_get_wtime();
                    
                    if (feats_down_size < 1)
                    {
                        ROS_WARN("No point, skip this scan!\n");

                        idx += time_seq[k];
                        continue;
                    }
                    const bool lidar_update_succeeded =
                        kf_input.update_iterated_dyn_share_modified();
                    record_current_lidar_pose(point_body.curvature);
                    if (!lidar_update_succeeded)
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }

                    solve_start = omp_get_wtime();

                    if (publish_odometry_without_downsample)
                    {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped);
                        if (runtime_pos_log)
                        {
                            euler_cur = SO3ToEuler(kf_input.x_.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " " << kf_input.x_.vel.transpose() \
                            <<" "<<kf_input.x_.bg.transpose()<<" "<<kf_input.x_.ba.transpose()<<" "<<kf_input.x_.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j); 
                    }
                    solve_time += omp_get_wtime() - solve_start;
                
                    update_time += omp_get_wtime() - t_update_start;
                    idx = idx + time_seq[k];
                }  
                }
                else
                {
                    sensor_msgs::Imu front;
                    if (imuBufferFront(front))
                    {
                    imu_last = imu_next;
                    imu_next = front;
                    while (imu_next.header.stamp.toSec() > time_current && ((imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte)))
                    { // >= ?
                        if (is_first_frame)
                        {
                            {
                                {
                                    sensor_msgs::Imu next;
                                    while (imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte)
                                    {
                                        if (!popImuFrontAndGetNext(next)) break;
                                        imu_last = imu_next;
                                        imu_next = next;
                                    }
                                }
                                
                                break;
                            }
                            imu_prop_cov = true;
                            
                            t_last = time_current;
                            time_update_last = time_current; 
                            input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            input_in.acc   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                            
                                is_first_frame = false;
                            
                        }
                        time_current = imu_next.header.stamp.toSec();

                        if (!is_first_frame)
                        {
                        double dt = time_current - t_last;

                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {        
                            // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = imu_next.header.stamp.toSec(); //time_current;
                        }
                        // kf_input.predict(dt, Q_input, input_in, true, false);

                        t_last = imu_next.header.stamp.toSec();
                    
                        input_in.gyro<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        input_in.acc<<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z; 
                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        sensor_msgs::Imu next;
                        if (!popImuFrontAndGetNext(next)) break;
                        imu_last = imu_next;
                        imu_next = next;
                        }
                        else
                        {
                            sensor_msgs::Imu next;
                            if (!popImuFrontAndGetNext(next)) break;
                            imu_last = imu_next;
                            imu_next = next;
                        }
                    }
                    }
                }
            }
            // M3D rot_cur_lidar;
            // {
            //     rot_cur_lidar = state.rot_end;
            // }
            // euler_cur = RotMtoEuler(rot_cur_lidar);
            // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            //                     (euler_cur(0), euler_cur(1), euler_cur(2));
            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample)
            {
                publish_odometry(pubOdomAftMapped);
            }

            /*** add the feature points to map ***/
            t3 = omp_get_wtime();
            
            if(feats_down_size > 4)
            {
                MapIncremental();
            }

            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            const double cloud_publish_start = omp_get_wtime();
            if (scan_world_publish_en || scan_world_downsample_publish_en ||
                pcd_save_en)
                publish_frame_world(pubLaserCloudFullRes, pubLaserCloudDownsampled);
            if (scan_bodyframe_pub_en)
                publish_frame_body(pubLaserCloudFullRes_body);
            const double cloud_publish_ms =
                (omp_get_wtime() - cloud_publish_start) * 1000.0;
            publish_runtime_metrics(runtime_metrics_pub,
                                    (t5 - t0) * 1000.0, cloud_publish_ms,
                                    match_time * 1000.0, solve_time * 1000.0,
                                    propag_time * 1000.0, update_time * 1000.0,
                                    (t3 - t1) * 1000.0, (t5 - t3) * 1000.0);
            
            /*** Debug variables Logging ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                {aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + update_time/frame_num;}
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + solve_time/frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1)/frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu, aver_time_icp, aver_time_propag); 
                fflush(stdout);
                if (!publish_odometry_without_downsample)
                {
                    if (!use_imu_as_input)
                    {
                        euler_cur = SO3ToEuler(kf_output.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " " << kf_output.x_.vel.transpose() \
                        <<" "<<kf_output.x_.omg.transpose()<<" "<<kf_output.x_.acc.transpose()<<" "<<kf_output.x_.gravity.transpose()<<" "<<kf_output.x_.bg.transpose()<<" "<<kf_output.x_.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
                    }
                    else
                    {
                        euler_cur = SO3ToEuler(kf_input.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " " << kf_input.x_.vel.transpose() \
                        <<" "<<kf_input.x_.bg.transpose()<<" "<<kf_input.x_.ba.transpose()<<" "<<kf_input.x_.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
        status = ros::ok();
        loop_rate.sleep();
    }
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    fout_out.close();
    fout_imu_pbp.close();
    return 0;
}
