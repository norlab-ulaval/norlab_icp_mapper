#include "VoxelDownsamplingMapperModule.h"

VoxelDownsamplingMapperModule::VoxelDownsamplingMapperModule(const PM::Parameters& params)
    : MapperModule("VoxelDownsamplingMapperModule", VoxelDownsamplingMapperModule::availableParameters(), params), isHashMapBuilt(false) {
    voxelSize = PM::Parametrizable::get<double>("voxelSize");
    pointsPerVoxel = PM::Parametrizable::get<std::size_t>("pointsPerVoxel");
}

PointMatcher<float>::DataPoints VoxelDownsamplingMapperModule::createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) {
    DataPoints outputMap(input);
    inPlaceCreateMap(outputMap, pose);
    return outputMap;
}

void VoxelDownsamplingMapperModule::inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) {
    DataPoints emptyMap = input.createSimilarEmpty();
    inPlaceUpdateMap(emptyMap, input, pose);
}

PointMatcher<float>::DataPoints VoxelDownsamplingMapperModule::updateMap(const PM::DataPoints& input, const PM::DataPoints& map,
                                                                         const PM::TransformationParameters& pose) {
    DataPoints outputMap(map);
    inPlaceUpdateMap(input, outputMap, pose);
    return outputMap;
}

void VoxelDownsamplingMapperModule::inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) {
    if (!isHashMapBuilt) {
        map = insertPointsInHashMap(map);
        isHashMapBuilt = true;
    } else {
        DataPoints insertedPoints = insertPointsInHashMap(input);
        map.concatenate(insertedPoints);
    }
}

const VoxelDownsamplingMapperModule::DataPoints VoxelDownsamplingMapperModule::insertPointsInHashMap(const PM::DataPoints& pointsToInsert) {
    int insertedPointsCount = 0;

    DataPoints points = pointsToInsert.createSimilarEmpty();

    for (int i = 0; i < pointsToInsert.getNbPoints(); ++i) {
        Voxel voxel = PointToVoxel(pointsToInsert.features.col(i), voxelSize);
        auto search = hashMap.find(voxel);

        if (search != hashMap.end()) {
            std::vector<Index>& voxel_points = search->second;
            if (voxel_points.size() < pointsPerVoxel) {
                voxel_points.emplace_back(i);

                points.setColFrom(insertedPointsCount, pointsToInsert, i);
                insertedPointsCount++;
            }
        } else {
            std::vector<Index> voxel_points;
            voxel_points.reserve(pointsPerVoxel);
            voxel_points.emplace_back(i);
            hashMap.insert({voxel, std::move(voxel_points)});

            points.setColFrom(insertedPointsCount, pointsToInsert, i);
            insertedPointsCount++;
        }
    }

    points.conservativeResize(insertedPointsCount);

    return points;
}
