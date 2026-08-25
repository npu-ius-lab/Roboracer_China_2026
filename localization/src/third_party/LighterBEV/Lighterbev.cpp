#include <torch/torch.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cctype>
#include "Lighterbev.h"
#include "rigid_ransac.hpp"

namespace {
struct KdPrefixView {
    const std::vector<std::vector<float>>& data;
    size_t count;  // 仅使用前 count 个

    KdPrefixView(const std::vector<std::vector<float>>& d, size_t c)
        : data(d), count(c) {}

    inline size_t kdtree_get_point_count() const { return count; }
    
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        return data[idx][dim];
    }

    // 关键：模板版本，适配不同 nanoflann 版本的 BBOX 实参类型
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};
} // namespace


LighterBEVManager::LighterBEVManager(std::string model_path,
                                     std::string device_preference) {
    setKeypointConfig(LighterBEVKeypointConfig());
    matcher_ = cv::makePtr<cv::BFMatcher>(cv::NORM_L2);
    model_path_ = model_path;
    device_preference_ = device_preference;
    std::transform(device_preference_.begin(), device_preference_.end(),
                   device_preference_.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (!loadmodel()) {
        throw std::runtime_error("Failed to initialize LighterBEV model: " + model_path_);
    }
    preheatModel();
}

void LighterBEVManager::setKeypointConfig(
    const LighterBEVKeypointConfig& config) {
    keypoint_backend_ = config.backend;
    std::transform(keypoint_backend_.begin(), keypoint_backend_.end(),
                   keypoint_backend_.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (keypoint_backend_ != "learned_fast" && keypoint_backend_ != "orb") {
        ROS_WARN("[LighterBEV] Unknown keypoint backend '%s'; using learned_fast.",
                 keypoint_backend_.c_str());
        keypoint_backend_ = "learned_fast";
    }

    fast_threshold_ = std::max(1, config.fast_threshold);
    fast_nonmax_suppression_ = config.fast_nonmax_suppression;
    fast_detector_ = cv::FastFeatureDetector::create(
        fast_threshold_, fast_nonmax_suppression_);

    const int orb_nfeatures = std::max(100, config.orb_nfeatures);
    const float orb_scale_factor = std::max(1.01f, config.orb_scale_factor);
    const int orb_nlevels = std::max(1, config.orb_nlevels);
    const int orb_edge_threshold = std::max(2, config.orb_edge_threshold);
    const int orb_patch_size = std::max(5, config.orb_patch_size);
    const int orb_fast_threshold = std::max(1, config.orb_fast_threshold);
    orb_detector_ = cv::ORB::create(
        orb_nfeatures, orb_scale_factor, orb_nlevels, orb_edge_threshold,
        0, 2, cv::ORB::HARRIS_SCORE, orb_patch_size, orb_fast_threshold);

    ROS_INFO("[LighterBEV] keypoint_backend=%s FAST(threshold=%d, nonmax=%s) "
             "ORB(nfeatures=%d, scale=%.2f, levels=%d, edge=%d, patch=%d, fast=%d)",
             keypoint_backend_.c_str(), fast_threshold_,
             fast_nonmax_suppression_ ? "true" : "false", orb_nfeatures,
             orb_scale_factor, orb_nlevels, orb_edge_threshold,
             orb_patch_size, orb_fast_threshold);
}

void LighterBEVManager::setMatchingParams(float ratio_thresh, int min_matches) {
    ratio_thresh_ = std::max(0.1f, std::min(ratio_thresh, 1.0f));
    min_matches_ = std::max(4, min_matches);
}

bool LighterBEVManager::loadmodel() {
    try {
        std::cout << "=> loading REIN state_dict from '" << model_path_ << "'" << std::endl;
        model_ = REIN();
        torch::Device device(torch::kCPU);
        if (device_preference_ == "cuda") {
            if (!torch::cuda::is_available()) {
                throw std::runtime_error(
                    "LighterBEV device is 'cuda', but CUDA is unavailable");
            }
            device = torch::Device(torch::kCUDA, 0);
        } else if (device_preference_ == "auto") {
            if (torch::cuda::is_available())
                device = torch::Device(torch::kCUDA, 0);
        } else if (device_preference_ != "cpu") {
            throw std::runtime_error(
                "Invalid LighterBEV device '" + device_preference_ +
                "' (expected cpu, cuda, or auto)");
        }
        model_->load_state_dict(model_path_);
        model_->to(device);
        model_->eval();
        std::cout << "LighterBEV inference device: "
                  << (device.is_cuda() ? "CUDA" : "CPU") << std::endl;
        initialized_ = true;
        return true;
    } catch (const c10::Error& e) {
        std::cerr << "Error loading the REIN model: " << e.what() << std::endl;
        ROS_ERROR("Error loading the model");
        initialized_ = false;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error loading the REIN model: " << e.what() << std::endl;
        ROS_ERROR("Error loading the model");
        initialized_ = false;
        return false;
    }
}

void LighterBEVManager::preheatModel() {
    if (!initialized_) {
        throw std::runtime_error("Cannot preheat LighterBEV before model initialization.");
    }
    // Create a random image for preheating 
    cv::Mat dummy_image(200, 200, CV_8UC3, cv::Scalar(127, 127, 127));

    // Preheat the model 100 times
    for (int i = 0; i < 10; ++i) {
        BEVFrame frame;
        frame.bev_img = dummy_image.clone(); // Use the dummy image for preheating
        detectBEVFeatures(frame, false); 
    }
    std::cout << "Model preheating completed." << std::endl;
}

bool LighterBEVManager::reload(const std::string &state_dict_path)
{
    if (!initialized_)
    {
        std::cerr << "[BEV] Failed to reload state dict: model is not initialized" << std::endl;
        return false;
    }
    try
    {
        torch::Device device = model_->parameters().front().device();
        model_->load_state_dict(state_dict_path);
        model_->to(device);
        model_->eval();
        std::cout << "[BEV] Reloaded state dict from " << state_dict_path << std::endl;
        return true;
    }
    catch (const c10::Error &e)
    {
        std::cerr << "[BEV] Failed to reload state dict: " << e.what() << std::endl;
        return false;
    }
}


torch::Tensor LighterBEVManager::cvMatToTensor(const cv::Mat& img)
{
    cv::Mat img_float;
    img.convertTo(img_float, CV_32F, 1.0 / 256.0);        

    if (img_float.channels() == 1)
        cv::cvtColor(img_float, img_float, cv::COLOR_GRAY2RGB);
    else
        cv::cvtColor(img_float, img_float, cv::COLOR_BGR2RGB);

    /* 关键：clone() 让 Tensor 拥有自己的连续内存，
       同时用 .contiguous() 保证布局符合 GPU 需求           */
    auto tensor_image = torch::from_blob(
            img_float.data,
            {img_float.rows, img_float.cols, 3},
            torch::TensorOptions().dtype(torch::kFloat32)).clone();   // <-- 这里！

    return tensor_image.permute({2, 0, 1})        // C,H,W
                    .unsqueeze(0)                 // 1,C,H,W
                    .contiguous();
}

void LighterBEVManager::detectBEVFeatures(BEVFrame& frame, bool save_local) {
    if (!initialized_) {
        throw std::runtime_error("LighterBEV model is not initialized.");
    }
    if (frame.bev_img.empty()) {
        throw std::runtime_error("LighterBEV input BEV image is empty.");
    }
    torch::Tensor img_tensor = cvMatToTensor(frame.bev_img);
    
    torch::NoGradGuard no_grad;
    torch::Device device = model_->parameters().front().device();
    torch::Tensor dev_in = img_tensor.to(device);

    auto tuple_output = model_->forward(dev_in);

    torch::Tensor output2 = std::get<1>(tuple_output); // local_feats
    torch::Tensor output3 = std::get<2>(tuple_output); // global_desc
    
    // A blocking device-to-CPU copy works on both CPU-only and CUDA builds.
    // It also guarantees that BEVFrame never keeps a tensor owned by a
    // device-specific stream after this function returns.
    if (save_local) {
        frame.local_feats = output2.detach().to(at::kCPU).contiguous();
    } else {
        frame.local_feats = torch::Tensor();
    }

    // Flatten the global descriptor and copy it to the CPU
    output3 = output3.view(-1); // conver to 2048
    auto output3_data = output3.detach().cpu(); // global_desc: numpy torch.Size([1, 2048])

    frame.global_desc.resize(output3_data.numel()); // Resize to 2048
    std::memcpy(frame.global_desc.data(), output3_data.data_ptr<float>(), output3_data.numel() * sizeof(float));
}


void LighterBEVManager::detectBEVFeaturesBatch(BEVFrame& src, BEVFrame& dst, bool save_local){
    if (!initialized_) {
        throw std::runtime_error("LighterBEV model is not initialized.");
    }
    // 0) 基础检查
    if (src.bev_img.empty() || dst.bev_img.empty()) {
        std::cerr << "[BEV] detectBEVFeaturesBatch: empty bev_img (src/dst)" << std::endl;
        return;
    }

    // 1) 各自转 Tensor: [1,3,H,W]（cvMatToTensor 内部已做 BGR->RGB 等）
    torch::Tensor t_src = cvMatToTensor(src.bev_img); // CPU float32
    torch::Tensor t_dst = cvMatToTensor(dst.bev_img); // CPU float32

    // 2) 拼 batch: [2,3,H,W]
    torch::Tensor batch = torch::cat({t_src, t_dst}, 0);


    // 3) 前向（无梯度；输入搬到模型所在设备）
    torch::Device device = model_->parameters().front().device();
    batch = batch.to(device);
    c10::InferenceMode no_grad_guard(true);
    auto tuple_output = model_->forward(batch);

    // 取输出（与单帧一致的 indices）：[2, C, H', W'] & [2, D] 或类似
    torch::Tensor out_local = std::get<1>(tuple_output); // local_feats
    torch::Tensor out_global = std::get<2>(tuple_output); // global_desc

    // 4) 拆出两帧的局部特征（可选）
    if (save_local) {
        // Blocking copies keep this path valid for CPU-only LibTorch too.
        src.local_feats = out_local.index({0}).detach().to(at::kCPU).contiguous();
        dst.local_feats = out_local.index({1}).detach().to(at::kCPU).contiguous();
    } else {
        src.local_feats = torch::Tensor();
        dst.local_feats = torch::Tensor();
    }

    // 5) 全局描述子：规整维度到 [2, D]
    //    有的模型可能输出 [2,1,D] / [2,D,1] / [2,D] 等，这里统一 flatten(1)
    if (out_global.dim() == 1) {
        out_global = out_global.view({2, -1});
    } else if (out_global.dim() >= 2) {
        out_global = out_global.flatten(1);
    }
    // 搬到 CPU
    auto gd_cpu = out_global.detach().to(at::kCPU).contiguous(); // [2, D]

    // 基本 sanity check
    if (gd_cpu.dim() != 2 || gd_cpu.size(0) != 2) {
        std::cerr << "[BEV] Unexpected global_desc shape: " << gd_cpu.sizes() << std::endl;
        return;
    }

}
void LighterBEVManager::getFAST(BEVFrame& frame) {
    // 0) 基本检查
    if (frame.bev_img.empty()) {
        std::cerr << "[BEV] getFAST: bev_img is empty\n";
        frame.query_descriptors.release();
        frame.keypoints.clear();
        return;
    }
    if (!frame.local_feats.defined()) {
        std::cerr << "[BEV] getFAST: local_feats is undefined (call detectBEVFeatures first)\n";
        frame.query_descriptors.release();
        frame.keypoints.clear();
        return;
    }

    // 1) FAST 关键点检测
    fast_detector_->detect(frame.bev_img, frame.keypoints);
    if (frame.keypoints.empty()) {
        frame.query_descriptors.release();
        return;
    }

    // 2) 获取特征图尺寸（局部特征）
    at::Tensor feats = frame.local_feats;
    if (feats.is_cuda()) feats = feats.cpu();
    feats = feats.contiguous(); // 转为连续内存

    const int64_t C = feats.size(0);
    const int Hf = feats.size(1);  // 特征图 H
    const int Wf = feats.size(2);  // 特征图 W

    // 3) 预分配描述子矩阵（N x C, CV_32F），逐行填充
    const int Nkp = static_cast<int>(frame.keypoints.size());
    cv::Mat desc_max(Nkp, static_cast<int>(C), CV_32F);
    std::vector<cv::KeyPoint> valid_keypoints;
    valid_keypoints.reserve(frame.keypoints.size());
    int valid_rows = 0;

    const int Himg = frame.bev_img.rows;
    const int Wimg = frame.bev_img.cols;
    const double scale_u = static_cast<double>(Wf) / static_cast<double>(Wimg);  // 图像到特征图坐标的比例
    const double scale_v = static_cast<double>(Hf) / static_cast<double>(Himg);

    // 逐个关键点获取描述子
    for (const auto& kp : frame.keypoints) {
        int u = static_cast<int>(std::round(kp.pt.x * scale_u)); // 转换到特征图坐标系
        int v = static_cast<int>(std::round(kp.pt.y * scale_v)); 
        if (u < 0 || u >= Wf || v < 0 || v >= Hf) continue; // 越界检查

        at::Tensor t = feats.index({torch::indexing::Slice(), v, u}); // 获取描述子
        t = t.contiguous(); // 转为连续内存

        // 填充到描述子矩阵
        float* row_ptr = desc_max.ptr<float>(valid_rows);
        std::memcpy(row_ptr, t.data_ptr<float>(), static_cast<size_t>(C) * sizeof(float));
        valid_keypoints.push_back(kp);
        ++valid_rows;
    }

    if (valid_rows == 0) {  // 没有有效描述子
        frame.query_descriptors.release();
        frame.keypoints.clear();
        return;
    }

    // 4) 截断到有效行（不做归一化）
    frame.query_descriptors = desc_max.rowRange(0, valid_rows).clone();
    frame.keypoints.swap(valid_keypoints);
}

void LighterBEVManager::getORB(BEVFrame& frame) {
    frame.keypoints.clear();
    frame.query_descriptors.release();
    if (frame.bev_img.empty()) {
        ROS_WARN("[LighterBEV] ORB input BEV image is empty.");
        return;
    }
    if (!orb_detector_) {
        ROS_ERROR("[LighterBEV] ORB detector is not configured.");
        return;
    }

    cv::Mat gray;
    if (frame.bev_img.channels() == 1) {
        gray = frame.bev_img;
    } else {
        cv::cvtColor(frame.bev_img, gray, cv::COLOR_BGR2GRAY);
    }
    orb_detector_->detectAndCompute(gray, cv::noArray(), frame.keypoints,
                                    frame.query_descriptors);
}



std::pair<Eigen::Matrix3d, int>
LighterBEVManager::matchFeatures(BEVFrame& frame, const BEVFrame& frame_prev) {
    // 1) 获取描述子
    cv::Mat query_descriptors = frame.query_descriptors;
    cv::Mat db_descriptors = frame_prev.query_descriptors;
    if (query_descriptors.empty() || db_descriptors.empty()) {
        return {Eigen::Matrix3d::Identity(), 0};
    }

    // Learned descriptors are float/L2. Outdoor ORB descriptors are
    // binary/Hamming and retain orientation invariance for reverse traversals.
    const int norm_type = query_descriptors.depth() == CV_8U
                              ? cv::NORM_HAMMING
                              : cv::NORM_L2;
    if (query_descriptors.type() != db_descriptors.type()) {
        ROS_WARN("[LighterBEV] Descriptor type mismatch: query=%d db=%d",
                 query_descriptors.type(), db_descriptors.type());
        return {Eigen::Matrix3d::Identity(), 0};
    }
    cv::BFMatcher matcher(norm_type, false);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(query_descriptors, db_descriptors, knn_matches, 2);  // k=2

    // 3) Lowe ratio test（与 Python 后端对齐）
    std::vector<cv::DMatch> all_match;
    for (const auto& mvec : knn_matches) {
        if (mvec.size() < 2) {
            continue;
        }
        const auto& first = mvec[0];
        const auto& second = mvec[1];
        if (first.distance < ratio_thresh_ * second.distance) {
            all_match.push_back(first);
        }
    }
    
    const int N = static_cast<int>(all_match.size());
    if (N < min_matches_) {
        ROS_INFO("[LighterBEV] backend=%s keypoints=%zu/%zu ratio_matches=%d < %d",
                 keypoint_backend_.c_str(), frame.keypoints.size(),
                 frame_prev.keypoints.size(), N, min_matches_);
        return {Eigen::Matrix3d::Identity(), 0};
    }

    Eigen::MatrixXd P1(N, 2), P2(N, 2);
    const int img_w = frame.bev_img.cols;
    const int img_h = frame.bev_img.rows;
    const double cx = 0.5 * static_cast<double>(img_w);
    const double cy = 0.5 * static_cast<double>(img_h);
    const double scale = metric_scale_;

    // 4) 将匹配的关键点位置转换为雷达尺度
    for (int i = 0; i < N; ++i) {
        const auto& m = all_match[i];
        const cv::Point2f& p1 = frame.keypoints[m.queryIdx].pt;
        const cv::Point2f& p2 = frame_prev.keypoints[m.trainIdx].pt;

        // 以图像中心为原点，缩放到“雷达尺度”
        P1(i, 0) = (cx - static_cast<double>(p1.x)) * scale;
        P1(i, 1) = (cy - static_cast<double>(p1.y)) * scale;
        P2(i, 0) = (cx - static_cast<double>(p2.x)) * scale;
        P2(i, 1) = (cy - static_cast<double>(p2.y)) * scale;
        

    }

    // 5) 刚体 RANSAC（只做一次）
    const int iters = std::max(50, max_iterations);

    Eigen::Matrix3d T12;
    std::vector<bool> inlier_mask;
    int ninliers = 0;
    
    // 调用 rigidRansac，确保返回的类型符合预期
    std::tie(T12, inlier_mask, ninliers) = rigidRansac(P1, P2, iters, ransac_threshold_);
    if (ninliers < min_matches_) {
        ROS_INFO("[LighterBEV] backend=%s keypoints=%zu/%zu ratio_matches=%d "
                 "ransac_inliers=%d < %d",
                 keypoint_backend_.c_str(), frame.keypoints.size(),
                 frame_prev.keypoints.size(), N, ninliers, min_matches_);
        return {Eigen::Matrix3d::Identity(), ninliers};
    }

    // 6) 根据 inlier_mask 过滤出内点（对应 good_matches 的子集）
    std::vector<cv::DMatch> inlier_matches;
    inlier_matches.reserve(static_cast<size_t>(ninliers));
    for (int i = 0; i < N; ++i) {
        if (i < static_cast<int>(inlier_mask.size()) && inlier_mask[i]) {
            inlier_matches.push_back(all_match[i]);
        }
    }

    // 7) 绘制匹配（只画内点）
    {
        cv::Mat img1 = frame.bev_img, img2 = frame_prev.bev_img;
        if (img1.channels() == 1) cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        if (img2.channels() == 1) cv::cvtColor(img2, img2, cv::COLOR_GRAY2BGR);

        cv::drawMatches(img1, frame.keypoints,
                        img2, frame_prev.keypoints,
                        inlier_matches, frame.img_matches,
                        cv::Scalar::all(-1),  // random colors make dense matches easier to inspect
                        cv::Scalar::all(-1),
                        std::vector<char>(),
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    }

    auto colorize_bev = [](const cv::Mat& gray, const cv::Scalar& color) {
        cv::Mat src = gray;
        if (src.channels() == 3) {
            cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
        }
        cv::Mat out(src.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        for (int y = 0; y < src.rows; ++y) {
            const uchar* in = src.ptr<uchar>(y);
            cv::Vec3b* dst = out.ptr<cv::Vec3b>(y);
            for (int x = 0; x < src.cols; ++x) {
                const uchar v = in[x];
                if (v == 0) {
                    continue;
                }
                dst[x][0] = static_cast<uchar>(std::min(255.0, color[0] * static_cast<double>(v) / 255.0));
                dst[x][1] = static_cast<uchar>(std::min(255.0, color[1] * static_cast<double>(v) / 255.0));
                dst[x][2] = static_cast<uchar>(std::min(255.0, color[2] * static_cast<double>(v) / 255.0));
            }
        }
        return out;
    };

    auto overlay_bev = [](const cv::Mat& base, const cv::Mat& add) {
        cv::Mat out = base.clone();
        for (int y = 0; y < out.rows; ++y) {
            cv::Vec3b* dst = out.ptr<cv::Vec3b>(y);
            const cv::Vec3b* src = add.ptr<cv::Vec3b>(y);
            for (int x = 0; x < out.cols; ++x) {
                if (src[x] == cv::Vec3b(0, 0, 0)) {
                    continue;
                }
                if (dst[x] == cv::Vec3b(0, 0, 0)) {
                    dst[x] = src[x];
                } else {
                    dst[x][0] = static_cast<uchar>(std::min(255, static_cast<int>(dst[x][0]) + static_cast<int>(src[x][0])));
                    dst[x][1] = static_cast<uchar>(std::min(255, static_cast<int>(dst[x][1]) + static_cast<int>(src[x][1])));
                    dst[x][2] = static_cast<uchar>(std::min(255, static_cast<int>(dst[x][2]) + static_cast<int>(src[x][2])));
                }
            }
        }
        return out;
    };

    auto plane_metric_to_pixel = [&](double mx, double my) {
        return cv::Point2f(static_cast<float>(cx - my / scale),
                           static_cast<float>(cy - mx / scale));
    };

    auto transform_pixel = [&](const cv::Point2f& p) {
        Eigen::Vector3d metric((cy - static_cast<double>(p.y)) * scale,
                               (cx - static_cast<double>(p.x)) * scale,
                               1.0);
        Eigen::Vector3d warped = T12 * metric;
        return plane_metric_to_pixel(warped.x(), warped.y());
    };

    const cv::Mat query_color = colorize_bev(frame.bev_img, cv::Scalar(255, 255, 255));
    const cv::Mat top1_color = colorize_bev(frame_prev.bev_img, cv::Scalar(0, 45, 255));
    frame.initial_overlap = overlay_bev(top1_color, query_color);

    std::vector<cv::Point2f> src_tri = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(img_w - 1), 0.0f),
        cv::Point2f(0.0f, static_cast<float>(img_h - 1))
    };
    std::vector<cv::Point2f> dst_tri = {
        transform_pixel(src_tri[0]),
        transform_pixel(src_tri[1]),
        transform_pixel(src_tri[2])
    };
    cv::Mat affine = cv::getAffineTransform(src_tri, dst_tri);
    cv::Mat warped_query;
    cv::warpAffine(query_color, warped_query, affine, frame_prev.bev_img.size(),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    frame.registered_overlap = overlay_bev(top1_color, warped_query);

    ROS_INFO_STREAM("[LighterBEV] backend=" << keypoint_backend_
                    << " matches=" << N
                    << " inliers=" << ninliers
                    << " scale=" << scale
                    << " plane_tx=" << T12(0, 2)
                    << " plane_ty=" << T12(1, 2));

    return {T12, ninliers};
}


std::pair<Eigen::Matrix4d, int> LighterBEVManager::poseEstimation(BEVFrame& src, BEVFrame& dst) {
    if (keypoint_backend_ == "orb") {
        // Global retrieval still uses the learned LighterBEV descriptor. ORB
        // is used only for rotation-aware local pose verification outdoors.
        getORB(src);
        getORB(dst);
    } else {
        detectBEVFeaturesBatch(src, dst, true);
        getFAST(src);
        getFAST(dst);
    }
    std::pair<Eigen::Matrix3d,int> bev_pose_pair = matchFeatures(src, dst);  
    Eigen::Matrix3d T2d = bev_pose_pair.first;
    int ninliers         = bev_pose_pair.second;
    Eigen::Matrix4d T4d = Eigen::Matrix4d::Identity();

    // rigidRansac estimates in plane metric [center-y, center-x]. With the
    // current BEV projector (swap_xy=true, flip_x=true, flip_y=true), this is
    // approximately [-y, -x] in the LiDAR local frame.
    Eigen::Matrix2d plane_to_local;
    plane_to_local << 0.0, -1.0,
                     -1.0,  0.0;
    Eigen::Matrix2d R_plane = T2d.block<2, 2>(0, 0);
    Eigen::Vector2d t_plane(T2d(0, 2), T2d(1, 2));
    Eigen::Matrix2d R_local = plane_to_local * R_plane * plane_to_local;
    Eigen::Vector2d t_local = plane_to_local * t_plane;

    T4d.block<2,2>(0,0) = R_local;
    T4d(0,3) = t_local.x();
    T4d(1,3) = t_local.y();

    ROS_INFO_STREAM("[LighterBEV] local_init tx=" << T4d(0, 3)
                    << " ty=" << T4d(1, 3)
                    << " yaw_deg="
                    << std::atan2(T4d(1, 0), T4d(0, 0)) * 180.0 / M_PI
                    << " ninliers=" << ninliers);

    // Drop heavy locals to avoid keeping per-keyframe local features.
    src.local_feats = torch::Tensor();
    dst.local_feats = torch::Tensor();
    src.keypoints.clear();
    dst.keypoints.clear();
    src.query_descriptors.release();
    dst.query_descriptors.release();

    return {T4d, ninliers};
}

void LighterBEVManager::addNewKeyFrame(BEVFrame& frame){
    detectBEVFeatures(frame);    // todo 独立线程处理  

    global_DB.push_back(frame.global_desc);
    kf_times_.push_back(frame.header.stamp.toSec());
}

bool LighterBEVManager::addKeyFrameDescriptor(
    const std::vector<float>& descriptor, double timestamp) {
    if (descriptor.empty()) {
        return false;
    }
    if (!global_DB.empty() && descriptor.size() != global_DB.front().size()) {
        return false;
    }
    global_DB.push_back(descriptor);
    kf_times_.push_back(timestamp);
    return true;
}

std::vector<std::pair<int, float>>
LighterBEVManager::detectGlobalRelocalizationCandidates(
    const BEVFrame& frame, int top_k, float max_l2_distance) const {
    std::vector<std::pair<int, float>> candidates;
    if (top_k <= 0 || frame.global_desc.empty() || global_DB.empty()) {
        return candidates;
    }

    candidates.reserve(global_DB.size());
    for (size_t index = 0; index < global_DB.size(); ++index) {
        const auto& descriptor = global_DB[index];
        if (descriptor.size() != frame.global_desc.size()) {
            continue;
        }
        double squared_distance = 0.0;
        for (size_t dim = 0; dim < descriptor.size(); ++dim) {
            const double delta = static_cast<double>(frame.global_desc[dim]) -
                                 static_cast<double>(descriptor[dim]);
            squared_distance += delta * delta;
        }
        const float distance = static_cast<float>(std::sqrt(squared_distance));
        if (std::isfinite(distance) &&
            (max_l2_distance <= 0.0f || distance <= max_l2_distance)) {
            candidates.emplace_back(static_cast<int>(index), distance);
        }
    }

    const size_t keep = std::min(candidates.size(),
                                 static_cast<size_t>(top_k));
    std::partial_sort(candidates.begin(), candidates.begin() + keep,
                      candidates.end(),
                      [](const auto& lhs, const auto& rhs) {
                          return lhs.second < rhs.second;
                      });
    candidates.resize(keep);
    return candidates;
}


int LighterBEVManager::detectLoopClosureIDGivenScan(const BEVFrame& frame){
    const double curr_time = frame.header.stamp.toSec();
    
    last_loop_id_  = -1;
    
    const auto& q_raw = frame.global_desc;
    const size_t N = global_DB.size();

    // ---------- 1) 计算候选前缀 cutoff：同时满足“时间间隔”和“ID 间隔” ----------
    const size_t NPOS = std::numeric_limits<size_t>::max();
    
    if (N <= static_cast<size_t>(keyframes_to_exclude)) {
        std::cout << "没有足够关键帧" << std::endl;
        return -1;
    }
    const size_t id_limit = N - 1 - static_cast<size_t>(keyframes_to_exclude);

    // 1.2 再叠加时间间隔（时间戳单调递增时可 break）
    size_t cutoff = NPOS;
    if (kf_times_.size() == N) {
        const double thr = curr_time - time_to_exclude;   // 允许的最晚时间
        for (size_t i = 0; i <= id_limit; ++i) {
            if (kf_times_[i] <= thr) {   // 等价于 curr_time - kf_times_[i] >= time_to_exclude
                cutoff = i;              // 继续找更靠后的“久远”帧
            } else {
                break;                   // 之后都更近了，直接停止
            }
        }
    } else {
        cutoff = id_limit;
    }
    
    if (cutoff == NPOS) {
        std::cout << " 没有候选关键帧 cutoff" << cutoff << "关键帧个数 "<< N << "时间数量 "<< kf_times_.size() << std::endl;
        return -1;  
    }
    // 区间是0，cutoff
    
    const size_t D = q_raw.size();
    const size_t prefix_count = static_cast<size_t>(cutoff) + 1;

    // 使用文件作用域的视图类型
    KdPrefixView view(global_DB, prefix_count);

    // L2 距离 + 固定维度
    using DistT  = nanoflann::L2_Simple_Adaptor<float, KdPrefixView>;
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<DistT, KdPrefixView, 2048>;

    KDTree index(D, view, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    index.buildIndex();

    size_t nn_idx = size_t(-1);
    float  dist2  = std::numeric_limits<float>::infinity();
    nanoflann::KNNResultSet<float> rs(1);
    rs.init(&nn_idx, &dist2);
    index.findNeighbors(rs, q_raw.data(), nanoflann::SearchParams(32, 0.0f, true));

    if (nn_idx == size_t(-1)) {
        std::cerr << "[BEV] 最近邻搜索失败：nn_idx == -1，prefix_count=" << prefix_count
                << ", DB size=" << global_DB.size() << std::endl;
        return -1;
    }

    last_loop_id_ = static_cast<int>(nn_idx);

    // 计算欧氏距离（非平方）
    const float l2 = std::sqrt(std::max(0.0f, dist2));

    if (l2 < pr_threshold) {
        // 打印距离
        std::cout << "特征距离（L2^2）= " << dist2 << ", 特征距离（L2）= " << l2
                << ", 最近邻ID = " << last_loop_id_ << std::endl;
        std::cout << "找到闭环候选：ID = " << last_loop_id_
                << "，L2 = " << l2 << "（L2^2 = " << dist2 << "）" << std::endl;

        return last_loop_id_;
    }else{
        return -1;
    }



    
}
