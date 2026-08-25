#pragma once

#include <Eigen/Core>

#include <memory>
#include <vector>

#include "OctVoxMap/OctVoxMap.hpp"
#include "map_backend/local_map_backend.h"

// Super-LIO OctVox/HKNN local map as a LocalMapBackend. Keeps the exact
// Point-LIO query contract: K fixed to 5, 5.0 m max range, and the same
// "reject when fewer than NUM_MATCH_POINTS neighbors" estimator behavior.
class OctVoxBackend : public LocalMapBackend
{
public:
    using OctPoint = Eigen::Vector3f;
    using OctPoints =
        std::vector<OctPoint, Eigen::aligned_allocator<OctPoint>>;
    using OctMap = LI2Sup::OctVoxMap<OctPoint, float>;

    OctVoxBackend(float resolution, std::size_t capacity)
        : resolution_(resolution),
          capacity_(capacity),
          map_(std::make_shared<OctMap>(
              typename OctMap::Options(resolution, capacity)))
    {
    }

    bool getClosestPoint(const PointType &query,
                         PointVector &neighbors,
                         int max_num,
                         double max_range) override
    {
        neighbors.clear();
        if (max_num != 5)
            return false;

        typename OctMap::KNNHeapType heap;
        heap.reset();
        map_->getTopK(OctPoint(query.x, query.y, query.z), heap);

        const float max_range2 =
            static_cast<float>(max_range * max_range);
        for (uint8_t i = 0; i < heap.count; ++i)
        {
            if (heap.dist2_[i] > max_range2)
                continue;
            PointType p;
            p.x = heap.points_[i].x();
            p.y = heap.points_[i].y();
            p.z = heap.points_[i].z();
            neighbors.emplace_back(p);
        }
        return !neighbors.empty();
    }

    void addPoints(const PointVector &points) override
    {
        OctPoints pts;
        pts.reserve(points.size());
        for (const auto &p : points)
            pts.emplace_back(p.x, p.y, p.z);
        map_->insert(pts);
    }

    void clear() override
    {
        map_.reset(new OctMap(typename OctMap::Options(resolution_,
                                                       capacity_)));
    }

    std::size_t numCells() const override
    {
        return map_->numVoxels();
    }

    std::size_t numRepresentatives() const override
    {
        return map_->numRepresentatives();
    }

    const char *name() const override
    {
        return "octvox";
    }

private:
    float resolution_;
    std::size_t capacity_;
    std::shared_ptr<OctMap> map_;
};
