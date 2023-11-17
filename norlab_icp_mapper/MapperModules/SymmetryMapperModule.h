//
// Created by Matěj Boxan on 2023-11-15.
//

#ifndef NORLAB_ICP_MAPPER_SYMMETRYMAPPERMODULE_H
#define NORLAB_ICP_MAPPER_SYMMETRYMAPPERMODULE_H

#include "MapperModule.h"

class SymmetryMapperModule: public MapperModule
{
    typedef PointMatcher<float> PM;
    typedef typename PM::DataPoints DataPoints;
	typedef PointMatcherSupport::Parametrizable P;

    std::shared_ptr<PM::DataPointsFilter> symmetryFilter;
	const float vrs;
	const float vro;
	const float dt;
	const float ct;
    const float initialVariance;
	const unsigned knn;

public:
    inline static const std::string description()
    {
        return "Lossy point cloud compression using incremental statistics.\n"
               "Combines nearby points into larger distributions,\n"
               "is especially useful for compressing large surfaces while preserving small detailed objects, e.g. leaves in tree canopies\n"
               "Required descriptors: none.\n"
               "Produced descriptors:  omega, deviation.\n"
               "Altered descriptors:  all.\n"
               "Altered features:     points coordinates and number of points.";
    }

    inline static const PM::ParametersDoc availableParameters()
    {
        return {
			{"vrs", "volume ratio for symmetry sampling", "5", "0", "inf", &P::Comp<float>},
			{"vro", "volume ratio for overlap sampling", "1.025", "0", "inf", &P::Comp<float>},
			{"dt", "distance threshold [m] for symmetry sampling", "0.05", "0", "inf", &P::Comp<float>},
			{"ct", "compressions tolerance [%]", "0.95", "0", "1", &P::Comp<float>},
			{"knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned>},
			{"initialVariance", "Variance on individual point positions (isotropic)", "0.0009", "0", "inf", &P::Comp<float>},};
    }

    explicit SymmetryMapperModule(const PM::Parameters& params = PM::Parameters());
    ~SymmetryMapperModule() override = default;

    PM::DataPoints createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose) override;
    void inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose) override;

    DataPoints updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose) override;
    void inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose) override;
};

#endif //NORLAB_ICP_MAPPER_SYMMETRYMAPPERMODULE_H
