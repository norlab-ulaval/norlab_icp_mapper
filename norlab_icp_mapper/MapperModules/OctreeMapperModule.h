//
// Created by Matěj Boxan on 2023-11-15.
//

#ifndef NORLAB_ICP_MAPPER_OCTREEMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_OCTREEMAPPERMODULE_H

#include "MapperModule.h"

class OctreeMapperModule: public MapperModule
{
    typedef PointMatcher<float> PM;
    typedef typename PM::DataPoints DataPoints;
	typedef PointMatcherSupport::Parametrizable P;

    std::shared_ptr<PM::DataPointsFilter> octreeFilter;
    const bool buildParallel;
    const float maxSizeByNode;
    const int samplingMethod;

public:
    //FIXME replace static methods by calls directly to libpointmatcher's datapointfilter
    inline static const std::string description()
    {
        return "Construct an Octree grid representation of the point cloud. Constructed by limiting the size of the bounding box. Down-sample by taking either the first or a random point, or compute the centroid.";
	}

    inline static const PM::ParametersDoc availableParameters()
    {
        return {
                {"buildParallel", "If 1 (true), use threads to build the octree.", "1", "0", "1", P::Comp<bool>},
                {"maxSizeByNode", "Size of the bounding box under which the octree stop dividing.", "0.2", "0", "+inf", &P::Comp<float>},
                {"samplingMethod", "Method to sample the Octree: First Point (0), Random (1), Centroid (2) (more accurate but costly), Medoid (3) (more accurate but costly)", "0", "0", "3", &P::Comp<int>}		};
    }

    explicit OctreeMapperModule(const PM::Parameters& params = PM::Parameters());
    ~OctreeMapperModule() override = default;

    PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) override;
    void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) override;

    DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) override;
    void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) override;
};

#endif //NORLAB_ICP_MAPPER_OCTREEMAPPERMODULE_H
