//
// Created by lfc on 2021/2/28.
//

#include "livox_laser_simulation/livox_points_plugin.h"
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"

namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (auto &data : datas) {
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;  //转化成标准的右手系角度
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
        return;
    }
    sdfPtr = sdf;
    auto rayElem = sdfPtr->GetElement("ray");
    auto rangeElem = rayElem->GetElement("range");

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "livox_laser_simulation",
                  ros::init_options::NoSigintHandler);
    }
    const auto livox_topic = sdf->Get<std::string>("ros_topic");
    const auto body_topic = sdf->Get<std::string>("pointcloud_topic");
    if (sdf->HasElement("frame_id")) frameId = sdf->Get<std::string>("frame_id");
    if (sdf->HasElement("body_frame_id")) bodyFrameId = sdf->Get<std::string>("body_frame_id");
    if (sdf->HasElement("body_offset_x")) bodyOffsetX = sdf->Get<double>("body_offset_x");
    if (sdf->HasElement("body_offset_y")) bodyOffsetY = sdf->Get<double>("body_offset_y");
    if (sdf->HasElement("body_offset_z")) bodyOffsetZ = sdf->Get<double>("body_offset_z");
    ROS_INFO_STREAM("Livox topics: " << livox_topic << " and " << body_topic);
    rosNode.reset(new ros::NodeHandle);
    livoxPointPub = rosNode->advertise<livox_ros_driver::CustomMsg>(livox_topic, 2);
    bodyCloudPub = rosNode->advertise<sensor_msgs::PointCloud2>(body_topic, 2);

    raySensor = _parent;
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
    maxPointSize = aviaInfos.size();

    RayPlugin::Load(_parent, sdfPtr);
    parentEntity = this->world->EntityByName(_parent->ParentName());
    auto physics = world->Physics();
    laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_parent->Pose());
    laserCollision->SetInitialRelativePose(_parent->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    samplesStep = sdfPtr->Get<int>("samples");
    downSample = sdfPtr->Get<int>("downsample");
    if (downSample < 1) {
        downSample = 1;
    }
    ROS_INFO_STREAM("sample:" << samplesStep);
    ROS_INFO_STREAM("downsample:" << downSample);
    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();
    minDist = rangeElem->Get<double>("min");
    maxDist = rangeElem->Get<double>("max");
    auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample) {
        int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos();
        end_point = maxDist * axis + offset.Pos();
        rayShape->AddRay(start_point, end_point);
    }
}

void LivoxPointsPlugin::OnNewLaserScans() {
    if (rayShape) {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        const ros::Time stamp = ros::Time::now();
        livox_ros_driver::CustomMsg livox_msg;
        livox_msg.header.stamp = stamp;
        livox_msg.header.frame_id = frameId;
        livox_msg.timebase = stamp.toNSec();
        livox_msg.lidar_id = 1;
        livox_msg.points.reserve(points_pair.size());

        struct BodyPoint { float x, y, z, intensity; };
        std::vector<BodyPoint> body_points;
        body_points.reserve(points_pair.size());

        for (auto &pair : points_pair) {
            const double range = rayShape->GetRange(pair.first);
            if (range >= RangeMax() || range <= RangeMin()) continue;
            const double intensity = rayShape->GetRetro(pair.first);
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, pair.second.zenith,
                                               pair.second.azimuth));
            const auto point = range * (ray * ignition::math::Vector3d(1.0, 0.0, 0.0));

            livox_ros_driver::CustomPoint custom_point;
            // Gazebo evaluates one instantaneous frame; non-zero synthetic
            // offsets would make Point-LIO deskew motion that is not present.
            custom_point.offset_time = 0;
            custom_point.x = point.X();
            custom_point.y = point.Y();
            custom_point.z = point.Z();
            custom_point.reflectivity = static_cast<uint8_t>(
                std::max(1.0, std::min(255.0, intensity)));
            custom_point.tag = 0;
            custom_point.line = static_cast<uint8_t>(pair.first % 4);
            livox_msg.points.push_back(custom_point);
            body_points.push_back({
                static_cast<float>(point.X() + bodyOffsetX),
                static_cast<float>(point.Y() + bodyOffsetY),
                static_cast<float>(point.Z() + bodyOffsetZ),
                static_cast<float>(intensity)});
        }

        livox_msg.point_num = livox_msg.points.size();
        livoxPointPub.publish(livox_msg);

        sensor_msgs::PointCloud2 body_cloud;
        body_cloud.header.stamp = stamp;
        body_cloud.header.frame_id = bodyFrameId;
        sensor_msgs::PointCloud2Modifier modifier(body_cloud);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(body_points.size());
        sensor_msgs::PointCloud2Iterator<float> out_x(body_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(body_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(body_cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> out_i(body_cloud, "intensity");
        for (const auto &point : body_points) {
            *out_x = point.x; *out_y = point.y; *out_z = point.z; *out_i = point.intensity;
            ++out_x; ++out_y; ++out_z; ++out_i;
        }
        bodyCloudPub.publish(body_cloud);
        ros::spinOnce();
    }
}

void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                       boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape) {
    auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision->RelativePose();
    int64_t end_index = currStartIndex + samplesStep;
    int ray_index = 0;
    auto ray_size = rays.size();
    points_pair.reserve(rays.size());
    for (int k = currStartIndex; k < end_index; k += downSample) {
        auto index = k % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos();
        end_point = maxDist * axis + offset.Pos();
        if (ray_index < ray_size) {
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }
    currStartIndex += samplesStep;
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
    if (rayShape)
        return rayShape->MinAngle();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->MaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

double LivoxPointsPlugin::RangeMin() const {
    if (rayShape)
        return rayShape->GetMinRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

double LivoxPointsPlugin::RangeMax() const {
    if (rayShape)
        return rayShape->GetMaxRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

double LivoxPointsPlugin::RangeResolution() const {
    if (rayShape)
        return rayShape->GetResRange();
    else
        return -1;
}

int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

int LivoxPointsPlugin::RayCount() const {
    if (rayShape)
        return rayShape->GetSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

int LivoxPointsPlugin::RangeCount() const {
    if (rayShape)
        return rayShape->GetSampleCount() * rayShape->GetScanResolution();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

int LivoxPointsPlugin::VerticalRayCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

int LivoxPointsPlugin::VerticalRangeCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
    } else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

double LivoxPointsPlugin::VerticalAngleResolution() const {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}
}
