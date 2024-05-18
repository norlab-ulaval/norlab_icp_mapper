//
// Created by Matěj Boxan on 2023-11-15.
//

#include "OctreeMapperModule.h"

OctreeMapperModule::OctreeMapperModule(const PM::Parameters& params):
	MapperModule("OctreeMapperModule",OctreeMapperModule::availableParameters(), params)
    {
        octreeFilter = PM::get().DataPointsFilterRegistrar.create(name, params);
    }

PointMatcher<float>::DataPoints OctreeMapperModule::createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose)
{
    DataPoints outputMap(input);
    inPlaceCreateMap(outputMap, pose);
    return outputMap;
}

void OctreeMapperModule::inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose)
{
    DataPoints emptyMap = input.createSimilarEmpty();
    inPlaceUpdateMap(emptyMap, input, pose);
}

PointMatcher<float>::DataPoints OctreeMapperModule::updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose)
{
    DataPoints outputMap(map);
    inPlaceUpdateMap(input, outputMap, pose);
    return outputMap;
}

void OctreeMapperModule::inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose)
{
    map.concatenate(input);
    octreeFilter->inPlaceFilter(map);
}
