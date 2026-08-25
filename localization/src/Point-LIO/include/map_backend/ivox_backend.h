#pragma once

#include <ivox/ivox3d.h>

#include <memory>

#include "map_backend/local_map_backend.h"

// Thin wrapper over the existing iVox hash-voxel map. It deliberately
// preserves the exact GetClosestPoint / AddPoints semantics so the baseline
// trajectory and timing are unchanged.
template <typename VoxelType>
class IVoxBackend : public LocalMapBackend
{
public:
    using Options = typename VoxelType::Options;

    explicit IVoxBackend(const Options &options)
        : options_(options), ivox_(std::make_shared<VoxelType>(options))
    {
    }

    bool getClosestPoint(const PointType &query,
                         PointVector &neighbors,
                         int max_num,
                         double max_range) override
    {
        return ivox_->GetClosestPoint(query, neighbors, max_num, max_range);
    }

    void addPoints(const PointVector &points) override
    {
        ivox_->AddPoints(points);
    }

    void clear() override
    {
        ivox_.reset(new VoxelType(options_));
    }

    std::size_t numCells() const override
    {
        return ivox_->NumValidGrids();
    }

    std::size_t numRepresentatives() const override
    {
        return ivox_->NumPoints();
    }

    const char *name() const override
    {
        return "ivox";
    }

private:
    Options options_;
    std::shared_ptr<VoxelType> ivox_;
};
