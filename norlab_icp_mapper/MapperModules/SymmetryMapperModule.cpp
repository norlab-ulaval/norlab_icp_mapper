//
// Created by Matěj Boxan on 2023-11-15.
//

#include "SymmetryMapperModule.h"

SymmetryMapperModule::SymmetryMapperModule(const PM::Parameters& params):
	MapperModule("SymmetryMapperModule",SymmetryMapperModule::availableParameters(), params),
        vrs(Parametrizable::get<float>("vrs")),
        vro(Parametrizable::get<float>("vro")),
        dt(Parametrizable::get<float>("dt")),
        ct(Parametrizable::get<float>("ct")),
        initialVariance(Parametrizable::get<float>("initialVariance")),
        knn(Parametrizable::get<unsigned>("knn"))
    {
        symmetryFilter = PM::get().DataPointsFilterRegistrar.create("SymmetryDataPointsFilter", params);
    }

PointMatcher<float>::DataPoints SymmetryMapperModule::createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose)
{
    DataPoints outputMap(input);
    inPlaceCreateMap(outputMap, pose);
    return outputMap;
}

void SymmetryMapperModule::inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose)
{
    symmetryFilter->inPlaceFilter(input);
    assert(input.descriptorExists("omega"));
    assert(input.descriptorExists("deviation"));
}

PointMatcher<float>::DataPoints SymmetryMapperModule::updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose)
{
    DataPoints outputMap(map);
    inPlaceUpdateMap(input, outputMap, pose);
    return outputMap;
}

void SymmetryMapperModule::inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose)
{
    map.concatenate(input);
    assert(map.descriptorExists("omega"));
    assert(map.descriptorExists("deviation"));
    symmetryFilter->inPlaceFilter(map);
}
