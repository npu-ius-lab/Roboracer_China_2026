#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace lighterbev_descriptor_io
{

struct DescriptorRecord
{
    double timestamp = 0.0;
    std::vector<float> descriptor;
};

inline bool save(const std::string &path,
                 const std::vector<DescriptorRecord> &records,
                 double bev_range,
                 double bev_resolution,
                 bool downsample_enable,
                 double downsample_voxel_size,
                 bool z_max_filter_enable,
                 double z_max)
{
    if (records.empty() || records.front().descriptor.empty() ||
        !std::isfinite(bev_range) || !std::isfinite(bev_resolution) ||
        !std::isfinite(downsample_voxel_size) || !std::isfinite(z_max) ||
        bev_range <= 0.0 || downsample_voxel_size <= 0.0 ||
        bev_resolution <= 0.0)
        return false;

    const uint32_t count = static_cast<uint32_t>(records.size());
    const uint32_t dimension =
        static_cast<uint32_t>(records.front().descriptor.size());
    for (const auto &record : records)
    {
        if (record.descriptor.size() != dimension)
            return false;
    }

    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream)
        return false;

    // Version 4 records all geometry-affecting BEV preprocessing. Descriptors
    // generated with different settings must never be paired with the query.
    const char magic[8] = {'L', 'B', 'E', 'V', 'D', 'B', '4', '\0'};
    const uint32_t version = 4;
    const uint32_t downsample_enabled = downsample_enable ? 1U : 0U;
    const uint32_t filter_enabled = z_max_filter_enable ? 1U : 0U;
    stream.write(magic, sizeof(magic));
    stream.write(reinterpret_cast<const char *>(&version), sizeof(version));
    stream.write(reinterpret_cast<const char *>(&count), sizeof(count));
    stream.write(reinterpret_cast<const char *>(&dimension), sizeof(dimension));
    stream.write(reinterpret_cast<const char *>(&bev_range), sizeof(bev_range));
    stream.write(reinterpret_cast<const char *>(&bev_resolution),
                 sizeof(bev_resolution));
    stream.write(reinterpret_cast<const char *>(&downsample_enabled),
                 sizeof(downsample_enabled));
    stream.write(reinterpret_cast<const char *>(&downsample_voxel_size),
                 sizeof(downsample_voxel_size));
    stream.write(reinterpret_cast<const char *>(&filter_enabled),
                 sizeof(filter_enabled));
    stream.write(reinterpret_cast<const char *>(&z_max), sizeof(z_max));
    for (const auto &record : records)
    {
        stream.write(reinterpret_cast<const char *>(&record.timestamp),
                     sizeof(record.timestamp));
        stream.write(reinterpret_cast<const char *>(record.descriptor.data()),
                     dimension * sizeof(float));
    }
    return static_cast<bool>(stream);
}

inline bool load(const std::string &path,
                 std::vector<DescriptorRecord> &records,
                 double expected_bev_range,
                 double expected_bev_resolution,
                 bool expected_downsample_enable,
                 double expected_downsample_voxel_size,
                 bool expected_z_max_filter_enable,
                 double expected_z_max)
{
    records.clear();
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
        return false;

    char magic[8] = {};
    uint32_t version = 0;
    uint32_t count = 0;
    uint32_t dimension = 0;
    double bev_range = 0.0;
    double bev_resolution = 0.0;
    uint32_t downsample_enabled = 0;
    double downsample_voxel_size = 0.0;
    uint32_t filter_enabled = 0;
    double z_max = 0.0;
    stream.read(magic, sizeof(magic));
    stream.read(reinterpret_cast<char *>(&version), sizeof(version));
    stream.read(reinterpret_cast<char *>(&count), sizeof(count));
    stream.read(reinterpret_cast<char *>(&dimension), sizeof(dimension));
    stream.read(reinterpret_cast<char *>(&bev_range), sizeof(bev_range));
    stream.read(reinterpret_cast<char *>(&bev_resolution),
                sizeof(bev_resolution));
    stream.read(reinterpret_cast<char *>(&downsample_enabled),
                sizeof(downsample_enabled));
    stream.read(reinterpret_cast<char *>(&downsample_voxel_size),
                sizeof(downsample_voxel_size));
    stream.read(reinterpret_cast<char *>(&filter_enabled),
                sizeof(filter_enabled));
    stream.read(reinterpret_cast<char *>(&z_max), sizeof(z_max));
    const char expected[8] = {'L', 'B', 'E', 'V', 'D', 'B', '4', '\0'};
    if (!stream || std::memcmp(magic, expected, sizeof(magic)) != 0 ||
        version != 4 || count == 0 || dimension == 0 ||
        std::abs(bev_range - expected_bev_range) > 1.0e-9 ||
        std::abs(bev_resolution - expected_bev_resolution) > 1.0e-9 ||
        (downsample_enabled != 0U) != expected_downsample_enable ||
        std::abs(downsample_voxel_size - expected_downsample_voxel_size) >
            1.0e-9 ||
        (filter_enabled != 0U) != expected_z_max_filter_enable ||
        std::abs(z_max - expected_z_max) > 1.0e-9 ||
        count > 10000000U || dimension > 65536U)
        return false;

    records.resize(count);
    for (auto &record : records)
    {
        record.descriptor.resize(dimension);
        stream.read(reinterpret_cast<char *>(&record.timestamp),
                    sizeof(record.timestamp));
        stream.read(reinterpret_cast<char *>(record.descriptor.data()),
                    dimension * sizeof(float));
        if (!stream)
        {
            records.clear();
            return false;
        }
    }
    return true;
}

}  // namespace lighterbev_descriptor_io
