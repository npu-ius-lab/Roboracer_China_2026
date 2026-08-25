#pragma once

#include <common_lib.h>

#include <cstddef>

// Abstract realtime local-map backend used by the Point-LIO estimator.
// The estimator only needs KNN query and insertion; the concrete backend
// decides the data structure (iVox hash voxel or Super-LIO OctVox/HKNN).
class LocalMapBackend
{
public:
    virtual ~LocalMapBackend() = default;

    virtual bool getClosestPoint(const PointType &query,
                                 PointVector &neighbors,
                                 int max_num,
                                 double max_range) = 0;

    virtual void addPoints(const PointVector &points) = 0;

    virtual void clear() = 0;

    virtual std::size_t numCells() const = 0;

    virtual std::size_t numRepresentatives() const = 0;

    virtual const char *name() const = 0;
};
