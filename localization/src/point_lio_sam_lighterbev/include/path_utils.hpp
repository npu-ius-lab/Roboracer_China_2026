#pragma once

#include <string>

#include <boost/filesystem/path.hpp>

namespace point_lio_sam_lighterbev
{

// Configuration files may use paths relative to the ROS package. Absolute
// paths remain supported as explicit runtime overrides.
inline std::string resolvePackagePath(const std::string &package_root,
                                      const std::string &configured_path,
                                      const std::string &default_relative_path)
{
    boost::filesystem::path path = configured_path.empty()
        ? boost::filesystem::path(default_relative_path)
        : boost::filesystem::path(configured_path);
    if (path.is_relative())
        path = boost::filesystem::path(package_root) / path;
    return path.string();
}

}  // namespace point_lio_sam_lighterbev
