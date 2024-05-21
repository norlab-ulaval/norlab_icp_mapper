#ifndef NORLAB_ICP_MAPPER_POINTDISTANCEMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_POINTDISTANCEMAPPERMODULE_H

#include "MapperModule.h"

class PointDistanceMapperModule : public MapperModule
{
    typedef PointMatcher<float> PM;
    typedef typename PM::DataPoints DataPoints;
	typedef PM::Parametrizable P;

    const float minDistNewPoint;

public:
    inline static const std::string description()
    {
        return "Create a new map from the first scan."
               "Update the map by iterating over each scan, removing points that are too close to existing points in the map.";
    }

    inline static const PM::ParametersDoc availableParameters()
    {
        return {
                {"minDistNewPoint", "Distance from current map points under which a new point is not added to the map (in meters).", "0.03", "0.0", "inf", P::Comp<float>}
        };
    }

    explicit PointDistanceMapperModule(const PM::Parameters& params = PM::Parameters());
    ~PointDistanceMapperModule() override = default;

    PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) override;
    void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) override;

    DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) override;
    void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) override;
};

#endif //NORLAB_ICP_MAPPER_POINTDISTANCEMAPPERMODULE_H
