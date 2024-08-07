#ifndef NORLAB_ICP_MAPPER_OCTREEMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_OCTREEMAPPERMODULE_H

#include "MapperModule.h"
#include "utils/octree.h"

class OctreeMapperModule : public MapperModule {
    typedef PointMatcher<float> PM;
    typedef typename PM::DataPoints DataPoints;

    Octree<float> octree;

    int i = 0;
    bool octree_is_built = false;
    bool buildParallel;
    std::size_t maxPointByNode;
    float maxSizeByNode;

    

   public:
    inline static const std::string description() { return PM::get().DataPointsFilterRegistrar.getDescription(name); }

    inline static const PM::ParametersDoc availableParameters() { return PM::get().DataPointsFilterRegistrar.getAvailableParameters(name); }

    explicit OctreeMapperModule(const PM::Parameters& params = PM::Parameters());
    ~OctreeMapperModule() override = default;

    PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) override;
    void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) override;

    DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) override;
    void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) override;

   private:
    inline static const std::string name = "OctreeGridDataPointsFilter";
};

#endif  // NORLAB_ICP_MAPPER_OCTREEMAPPERMODULE_H
