#ifndef NORLAB_ICP_MAPPER_DYNAMICPOINTSMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_DYNAMICPOINTSMAPPERMODULE_H

#include "MapperModule.h"

/**
 * Implements Dynamic points removal from Pomerleau 2014
 */
class DynamicPointsMapperModule : public MapperModule {

    typedef PointMatcher<float> PM;
	typedef PM::Parametrizable P;
    typedef typename PM::DataPoints DataPoints;

    const float priorDynamic;
    const float thresholdDynamic;
    const float alpha;
    const float beta;
    const float beamHalfAngle;
    const float epsilonA;
    const float epsilonD;
    const float sensorMaxRange;

    // A function that transforms points and their descriptors between sensor and map frame.
    // Used in the inPlaceUpdateMap method
    std::shared_ptr<PM::Transformation> transformation;

public:
    inline static const std::string description()
    {
        return "Remove points that are identified as dynamic.";
    }

    inline static const PM::ParametersDoc availableParameters()
    {
        return {
                {"priorDynamic", "A priori probability of points being dynamic.", "0.6", "0.0", "1.0", P::Comp<float>},
                {"thresholdDynamic", "Probability at which a point is considered permanently dynamic.", "0.6", "0.0", "1.0", P::Comp<float>},
                {"alpha", "Probability of staying static given that the point was static.", "0.8", "0.0", "1.0", P::Comp<float>},
                {"beta", "Probability of staying dynamic given that the point was dynamic.", "0.99", "0.0", "1.0", P::Comp<float>},
                {"beamHalfAngle", "Half angle of the cones formed by the sensor laser beams (in rad).", "0.01", "0.0", "1.57079632679489661923132169163975144", P::Comp<float>},
                {"epsilonA", "Error proportional to the sensor distance.", "0.01", "0.0", "inf", P::Comp<float>},
                {"epsilonD", "Fix error on the sensor distance (in meters).", "0.01", "0.0", "inf", P::Comp<float>},
                {"sensorMaxRange", "Maximum reading distance of the laser (in meters).", "200", "0.0", "inf", P::Comp<float>},
        };
    }

    explicit DynamicPointsMapperModule(const PM::Parameters& params = PM::Parameters());
    ~DynamicPointsMapperModule() override = default;

    PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) override;
    void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) override;

    DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) override;
    void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) override;


    void convertToSphericalCoordinates(const PM::DataPoints &points, PM::Matrix &radii,
                                       PM::Matrix &angles) const;
};


#endif //NORLAB_ICP_MAPPER_DYNAMICPOINTSMAPPERMODULE_H
