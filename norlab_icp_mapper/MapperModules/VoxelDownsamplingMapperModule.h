#ifndef NORLAB_ICP_MAPPER_VOXELDOWNSAMPLINGMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_VOXELDOWNSAMPLINGMAPPERMODULE_H

#include <cstddef>
#include "MapperModule.h"
#include <unordered_map>

// got this hashing of eigen vector from kiss-icp : https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelUtils.hpp
using Voxel = Eigen::Vector3i;
template <>
struct std::hash<Voxel> {
    std::size_t operator()(const Voxel& voxel) const {
        const uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
        return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};

class VoxelDownsamplingMapperModule : public MapperModule {
    typedef PointMatcher<float> PM;
    typedef typename PM::DataPoints DataPoints;
    typedef PM::Parametrizable P;
    typedef DataPoints::Index Index;

   public:
    inline static const std::string description() { return "Downsample the map using a voxel grid."; }

    inline static const PM::ParametersDoc availableParameters() {
        return {
            {"voxelSize", "The size of the voxel.", "1.0", "0.0", "inf", P::Comp<float>},
            {"pointsPerVoxel", "The amount of points per voxel.", "3", "1", "9999999", P::Comp<size_t>},
        };
    }

    explicit VoxelDownsamplingMapperModule(const PM::Parameters& params = PM::Parameters());
    ~VoxelDownsamplingMapperModule() override = default;

    PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) override;
    void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) override;

    DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) override;
    void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) override;

   private:
    const DataPoints insertPointsInHashMap(const PM::DataPoints& pointsToInsert);

    inline Voxel PointToVoxel(const Eigen::Vector3f& point, const double voxel_size) {
        return Voxel(static_cast<int>(std::floor(point.x() / voxel_size)), static_cast<int>(std::floor(point.y() / voxel_size)),
                     static_cast<int>(std::floor(point.z() / voxel_size)));
    }

    size_t pointsPerVoxel;
    double voxelSize;
    bool isHashMapBuilt;
    std::unordered_map<Voxel, std::vector<Index>> hashMap;
};

#endif  // NORLAB_ICP_MAPPER_VOXELDOWNSAMPLINGMAPPERMODULE_H
