///// C++ common headers
#include <tuple>
#include <vector>
#include <memory>
#include <limits>
#include <iostream>

///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <std_msgs/Header.h>
#include <boost/make_shared.hpp>
///// Eigen
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#pragma once
///// coded headers
typedef Eigen::Matrix4d M4D;

struct BEVFrame {
    std_msgs::Header header;
    cv::Mat bev_img;
    pcl::PointCloud<pcl::PointXYZI>::Ptr points;
    M4D pose;
    torch::Tensor local_feats;
    std::vector<float> global_desc;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat query_descriptors;
    cv::Mat img_matches;
    cv::Mat initial_overlap;
    cv::Mat registered_overlap;
    BEVFrame clone() const {
        BEVFrame f;
        f.header  = header;
        f.bev_img = bev_img.clone();
        f.pose = pose;
        f.keypoints = keypoints;
        f.global_desc = global_desc; 
        f.img_matches = img_matches.clone();
        f.initial_overlap = initial_overlap.clone();
        f.registered_overlap = registered_overlap.clone();
        f.query_descriptors = query_descriptors.clone();
        if (local_feats.defined()) {
            f.local_feats = local_feats.clone();
        }
        f.points = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        if (points) {
            *(f.points) = *points;
        }
        return f;
    }
};

