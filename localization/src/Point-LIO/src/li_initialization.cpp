#include "li_initialization.h"

ros::Publisher recovery_cloud_pub;
double last_recovery_cloud_publish_time = -1.0;

namespace
{
// Publishes the raw preprocessed LiDAR cloud in the body frame using ONLY the
// fixed LiDAR->IMU/body extrinsic. No ESKF pose, no deskew, no multi-frame
// accumulation: LighterBEV recovery must stay independent of Point-LIO state.
void maybePublishRecoveryCloud(const PointCloudXYZI::Ptr &lidar_cloud,
                               const double stamp)
{
    if (!recovery_cloud_enable)
        return;
    if (recovery_cloud_max_hz <= 0.0)
        return;
    if (last_recovery_cloud_publish_time >= 0.0 &&
        stamp - last_recovery_cloud_publish_time <
            1.0 / recovery_cloud_max_hz)
        return;
    last_recovery_cloud_publish_time = stamp;

    PointCloudXYZI::Ptr body_cloud(new PointCloudXYZI());
    body_cloud->points.reserve(lidar_cloud->size());
    for (const auto &p : lidar_cloud->points)
    {
        const V3D p_lidar(p.x, p.y, p.z);
        const V3D p_body = Lidar_R_wrt_IMU * p_lidar + Lidar_T_wrt_IMU;
        PointType out;
        out.x = static_cast<float>(p_body(0));
        out.y = static_cast<float>(p_body(1));
        out.z = static_cast<float>(p_body(2));
        out.intensity = p.intensity;
        body_cloud->points.emplace_back(out);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*body_cloud, msg);
    msg.header.stamp = ros::Time().fromSec(stamp);
    msg.header.frame_id = "body";
    recovery_cloud_pub.publish(msg);
}
}  // namespace

bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0, online_calib_starts_time = 0.0; //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::Imu imu_last, imu_next;
// sensor_msgs::Imu::ConstPtr imu_last_ptr;
PointCloudXYZI::Ptr  ptr_con(new PointCloudXYZI());
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

condition_variable sig_buffer;
int scan_count = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
std::deque<double>               time_buffer;
std::deque<sensor_msgs::Imu::Ptr> imu_deque;
std::atomic<unsigned long> dropped_lidar_scans_total{0};

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    const double stamp = msg->header.stamp.toSec();
    if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        // Order check, timestamp update and both deque pushes must be atomic
        // relative to the other callbacks and to sync_packages().
        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            if (stamp < last_timestamp_lidar)
            {
                ROS_ERROR("lidar loop back, clear buffer");
                return;
            }
            last_timestamp_lidar = stamp;
            while (!ptr.empty() && !timestamp_lidar.empty()) {
                lidar_buffer.push_back(ptr.front());
                ptr.pop_front();
                time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
                timestamp_lidar.pop_front();
            }
        }
    }
    else
    {
        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(20000,1));
        p_pre->process(msg, ptr);
        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            if (stamp < last_timestamp_lidar)
            {
                ROS_ERROR("lidar loop back, clear buffer");
                return;
            }
            last_timestamp_lidar = stamp;
            if (con_frame)
            {
                if (frame_ct == 0)
                {
                    time_con = stamp; //msg->header.stamp.toSec();
                }
                if (frame_ct < 10)
                {
                    for (int i = 0; i < ptr->size(); i++)
                    {
                        ptr->points[i].curvature += (stamp - time_con) * 1000;
                        ptr_con->push_back(ptr->points[i]);
                    }
                    frame_ct ++;
                }
                else
                {
                    PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
                    *ptr_con_i = *ptr_con;
                    lidar_buffer.push_back(ptr_con_i);
                    double time_con_i = time_con;
                    time_buffer.push_back(time_con_i);
                    ptr_con->clear();
                    frame_ct = 0;
                }
            }
            else
            {
                if (ptr->points.size() > 0)
                {
                    lidar_buffer.emplace_back(ptr);
                    time_buffer.emplace_back(stamp);
                }
            }
        }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    const double stamp = msg->header.stamp.toSec();
    if (cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        for (size_t i = 0; i < ptr.size() && i < timestamp_lidar.size(); ++i)
            maybePublishRecoveryCloud(ptr[i], timestamp_lidar[i] / 1000.0);

        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            if (stamp < last_timestamp_lidar)
            {
                ROS_ERROR("lidar loop back, clear buffer");
                return;
            }
            last_timestamp_lidar = stamp;
            while (!ptr.empty() && !timestamp_lidar.empty()) {
                lidar_buffer.push_back(ptr.front());
                ptr.pop_front();
                time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
                timestamp_lidar.pop_front();
            }
        }
    }
    else
    {
        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
        p_pre->process(msg, ptr);
        maybePublishRecoveryCloud(ptr, stamp);
        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            if (stamp < last_timestamp_lidar)
            {
                ROS_ERROR("lidar loop back, clear buffer");
                return;
            }
            last_timestamp_lidar = stamp;
            if (con_frame)
            {
                if (frame_ct == 0)
                {
                    time_con = stamp; //msg->header.stamp.toSec();
                }
                if (frame_ct < 10)
                {
                    for (int i = 0; i < ptr->size(); i++)
                    {
                        ptr->points[i].curvature += (stamp - time_con) * 1000;
                        ptr_con->push_back(ptr->points[i]);
                    }
                    frame_ct ++;
                }
                else
                {
                    PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
                    *ptr_con_i = *ptr_con;
                    double time_con_i = time_con;
                    lidar_buffer.push_back(ptr_con_i);
                    time_buffer.push_back(time_con_i);
                    ptr_con->clear();
                    frame_ct = 0;
                }
            }
            else
            {
                if (ptr->points.size() > 0)
                {
                    lidar_buffer.emplace_back(ptr);
                    time_buffer.emplace_back(stamp);
                }
            }
        }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

    double timestamp = msg->header.stamp.toSec();
    // The comparison, timestamp update and push must be atomic so the deque
    // stays ordered even when AsyncSpinner runs several callbacks at once.
    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        if (timestamp < last_timestamp_imu)
        {
            ROS_ERROR("imu loop back, clear deque");
            return;
        }
        imu_deque.emplace_back(msg);
        last_timestamp_imu = timestamp;
    }
}

bool sync_packages(MeasureGroup &meas)
{
    // Bounded-latency catch-up. Only drop LiDAR scans here, before any scan
    // has been selected for the point-wise EKF update; the IMU deque is left
    // intact so the estimator can propagate across the dropped gap.
    dropStaleLidarFramesForCatchup();

    // This function only selects/pops shared buffer entries (no ICP/EKF
    // work), so one short critical section covers every read and mutation.
    std::lock_guard<std::mutex> lock(mtx_buffer);
    {
    if (!imu_en)
    {
        if (!lidar_buffer.empty())
        {
            if (!lidar_pushed)
            {
                meas.lidar = lidar_buffer.front();
                meas.lidar_beg_time = time_buffer.front();
                lose_lid = false;
                if(meas.lidar->points.size() < 1) 
                {
                    cout << "lose lidar" << std::endl;
                    // return false;
                    lose_lid = true;
                }
                else
                {
                    double end_time = meas.lidar->points.back().curvature;
                    for (auto pt: meas.lidar->points)
                    {
                        if (pt.curvature > end_time)
                        {
                            end_time = pt.curvature;
                        }
                    }
                    lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                    meas.lidar_last_time = lidar_end_time;
                }
                lidar_pushed = true;
            }
            
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            lidar_pushed = false;
            if (!lose_lid)
            {
                return true;
            }
            else
            {
                return false;
            }
        }        
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }
    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        lose_lid = false;
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if(meas.lidar->points.size() < 1) 
        {
            cout << "lose lidar" << endl;
            lose_lid = true;
            // lidar_buffer.pop_front();
            // time_buffer.pop_front();
            // return false;
        }
        else
        {
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            // cout << "check time lidar:" << end_time << endl;
            meas.lidar_last_time = lidar_end_time;
        }
        lidar_pushed = true;
    }

    if (!lose_lid && (last_timestamp_imu < lidar_end_time))
    {
        return false;
    }
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte)
    {
        return false;
    }

    if (!lose_lid && !imu_pushed)
    { 
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            double imu_time = imu_deque.front()->header.stamp.toSec();
            imu_next = *(imu_deque.front());
            while (imu_time < lidar_end_time)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                imu_next = *(imu_deque.front());
            }
        }
        imu_pushed = true;
    }

    if (lose_lid && !imu_pushed)
    { 
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            double imu_time = imu_deque.front()->header.stamp.toSec();

            imu_next = *(imu_deque.front());
            while (imu_time < meas.lidar_beg_time + lidar_time_inte)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                imu_next = *(imu_deque.front());
            }
        }
        imu_pushed = true;
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    return true;
    }
}

SensorBufferStats getSensorBufferStats()
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    SensorBufferStats stats;
    stats.latest_lidar_stamp = last_timestamp_lidar;
    stats.latest_imu_stamp = last_timestamp_imu;
    stats.lidar_queue_size = lidar_buffer.size();
    stats.imu_queue_size = imu_deque.size();
    if (!time_buffer.empty())
    {
        stats.lidar_front_stamp = time_buffer.front();
        stats.lidar_back_stamp = time_buffer.back();
    }
    return stats;
}

bool imuBufferEmpty()
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    return imu_deque.empty();
}

bool imuBufferFront(sensor_msgs::Imu &out)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (imu_deque.empty())
        return false;
    out = *imu_deque.front();
    return true;
}

bool imuBufferPopFront()
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (imu_deque.empty())
        return false;
    imu_deque.pop_front();
    return true;
}

bool popImuFrontAndGetNext(sensor_msgs::Imu &next)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (imu_deque.empty())
        return false;
    imu_deque.pop_front();
    if (imu_deque.empty())
        return false;
    next = *imu_deque.front();
    return true;
}

size_t imuBufferSize()
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    return imu_deque.size();
}

size_t dropStaleLidarFramesForCatchup()
{
    if (!enable_lidar_backlog_catchup)
        return 0;

    std::lock_guard<std::mutex> lock(mtx_buffer);
    // Once sync_packages() has selected its front scan, meas.lidar points at
    // that object until enough IMU data arrives. Dropping the queue front in
    // this state would make the later pop remove a different scan and corrupt
    // LiDAR/IMU synchronization.
    if (lidar_pushed)
        return 0;

    if (lidar_buffer.size() != time_buffer.size())
    {
        ROS_ERROR_THROTTLE(
            1.0,
            "[PointLIO CATCHUP] paired buffers inconsistent: lidar_q=%zu "
            "time_q=%zu; refusing to drop",
            lidar_buffer.size(), time_buffer.size());
        return 0;
    }
    if (lidar_buffer.size() <= min_lidar_frames_to_keep)
        return 0;

    const double initial_span = time_buffer.back() - time_buffer.front();
    if (initial_span <= lidar_backlog_trigger_s)
        return 0;

    size_t dropped = 0;
    while (lidar_buffer.size() > min_lidar_frames_to_keep &&
           time_buffer.size() == lidar_buffer.size())
    {
        const double span = time_buffer.back() - time_buffer.front();
        if (span <= lidar_backlog_target_s)
            break;
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        ++dropped;
    }

    dropped_lidar_scans_total.fetch_add(dropped, std::memory_order_relaxed);
    if (dropped > 0)
    {
        ROS_WARN_THROTTLE(1.0,
            "[PointLIO CATCHUP] dropped=%zu span_before=%.3f span_after=%.3f "
            "lidar_q=%zu imu_q=%zu",
            dropped, initial_span, time_buffer.back() - time_buffer.front(),
            lidar_buffer.size(), imu_deque.size());
    }
    return dropped;
}
