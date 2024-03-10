//
// Created by Matěj Boxan on 2024-03-10.
//

#ifndef NORLAB_ICP_MAPPER_COMPUTEDYNAMICSMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_COMPUTEDYNAMICSMAPPERMODULE_H

#include "MapperModule.h"

class ComputeDynamicsMapperModule : public MapperModule {

    typedef PointMatcher<float> PM;
    typedef typename PM::DataPoints DataPoints;

    const float priorDynamic;
    const float thresholdDynamic;
    const float alpha;
    const float beta;
    const float beamHalfAngle;
    const float epsilonA;
    const float epsilonD;
    const float sensorMaxRange;

    std::shared_ptr<PM::Transformation> transformation;

public:
    inline static const std::string description()
    {
        return "TODO";
    }

    inline static const PM::ParametersDoc availableParameters()
    {
        return {
                {"priorDynamic", "TODO", "0.6"},
                {"thresholdDynamic", "TODO", "0.6"},
                {"alpha", "TODO", "0.8"},
                {"beta", "TODO", "0.99"},
                {"beamHalfAngle", "TODO", "0.01"},
                {"epsilonA", "TODO", "0.01"},
                {"espilonD", "TODO", "0.01"},
                {"sensorMaxRange", "TODO", "200"},
        };
    }

    explicit ComputeDynamicsMapperModule(const PM::Parameters& params = PM::Parameters());
    ~ComputeDynamicsMapperModule() override = default;

    PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) override;
    void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) override;

    DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) override;
    void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) override;


    void convertToSphericalCoordinates(const PM::DataPoints &points, PM::Matrix &radii,
                                       PM::Matrix &angles) const;
};


#endif //NORLAB_ICP_MAPPER_COMPUTEDYNAMICSMAPPERMODULE_H
