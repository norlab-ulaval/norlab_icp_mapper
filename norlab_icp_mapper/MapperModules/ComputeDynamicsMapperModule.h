//
// Created by Matěj Boxan on 2024-03-10.
//

#ifndef NORLAB_ICP_MAPPER_COMPUTEDYNAMICSMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_COMPUTEDYNAMICSMAPPERMODULE_H

#include "MapperModule.h"

/**
 * Implements Dynamic points removal from Pomerleau 2014
 */
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
        return "Remove points that are identified as dynamic.";
    }

    inline static const PM::ParametersDoc availableParameters()
    {
        return {
                {"priorDynamic", "A priori probability of points being dynamic.", "0.6"},
                {"thresholdDynamic", "Probability at which a point is considered permanently dynamic.", "0.6"},
                {"alpha", "Probability of staying static given that the point was static.", "0.8"},
                {"beta", "Probability of staying dynamic given that the point was dynamic.", "0.99"},
                {"beamHalfAngle", "Half angle of the cones formed by the sensor laser beams (in rad).", "0.01"},
                {"epsilonA", "Error proportional to the sensor distance.", "0.01"},
                {"espilonD", "Fix error on the sensor distance (in meters).", "0.01"},
                {"sensorMaxRange", "Maximum reading distance of the laser (in meters).", "200"},
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
