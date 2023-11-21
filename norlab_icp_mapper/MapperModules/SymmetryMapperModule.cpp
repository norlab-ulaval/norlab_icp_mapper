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
    // TODO replace this by AddDescriptorDataPointsFilter (to be coded)
    PM::DataPoints input_enhanced(input);
    PM::Matrix omegas = PM::Matrix::Zero(1, input_enhanced.getNbPoints());
    omegas.setOnes();
    input_enhanced.addDescriptor("omega", omegas);

    unsigned dim = input_enhanced.getEuclideanDim();
    PM::Matrix deviations = PM::Matrix::Zero(std::pow(dim, 2), input_enhanced.getNbPoints());
    if(dim == 2)
    {
        deviations.row(0) = PM::Matrix::Constant(1, input_enhanced.getNbPoints(), initialVariance);
        deviations.row(3) = PM::Matrix::Constant(1, input_enhanced.getNbPoints(), initialVariance);
    }
    else
    {
        deviations.row(0) = PM::Matrix::Constant(1, input_enhanced.getNbPoints(), initialVariance);
        deviations.row(4) = PM::Matrix::Constant(1, input_enhanced.getNbPoints(), initialVariance);
        deviations.row(8) = PM::Matrix::Constant(1, input_enhanced.getNbPoints(), initialVariance);
    }

    input_enhanced.addDescriptor("deviation", deviations);

    map.concatenate(input_enhanced);
    assert(map.descriptorExists("omega"));
    assert(map.descriptorExists("deviation"));
    symmetryFilter->inPlaceFilter(map);
}
