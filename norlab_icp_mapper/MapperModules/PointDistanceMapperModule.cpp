#include <nabo/nabo.h>
#include "PointDistanceMapperModule.h"

PointDistanceMapperModule::PointDistanceMapperModule(const PM::Parameters& params):
	MapperModule("PointDistanceMapperModule",PointDistanceMapperModule::availableParameters(), params),
	    minDistNewPoint(PM::Parametrizable::get<float>("minDistNewPoint"))
    {}

PointMatcher<float>::DataPoints PointDistanceMapperModule::createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose)
{
    DataPoints outputMap(input);
    inPlaceCreateMap(outputMap, pose);
    return outputMap;
}

void PointDistanceMapperModule::inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose)
{
// keep the input untouched
}

PointMatcher<float>::DataPoints PointDistanceMapperModule::updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose)
{
    DataPoints outputMap(map);
    inPlaceUpdateMap(input, outputMap, pose);
    return outputMap;
}

void PointDistanceMapperModule::inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose)
{
	typedef Nabo::NearestNeighbourSearch<float> NNS;

	PM::Matches matches(PM::Matches::Dists(1, input.getNbPoints()), PM::Matches::Ids(1, input.getNbPoints()));
	std::shared_ptr<NNS> nns = std::shared_ptr<NNS>(NNS::create(map.features, map.features.rows() - 1,
																NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));

	nns->knn(input.features, matches.ids, matches.dists, 1, 0);

	int inputPointsToKeepPointCount = 0;
	PM::DataPoints inputPointsToKeep(input.createSimilarEmpty());
	for(int i = 0; i < input.getNbPoints(); ++i)
	{
		if(matches.dists(i) >= std::pow(minDistNewPoint, 2))
		{
			inputPointsToKeep.setColFrom(inputPointsToKeepPointCount, input, i);
			inputPointsToKeepPointCount++;
		}
	}
	inputPointsToKeep.conservativeResize(inputPointsToKeepPointCount);
    map.concatenate(inputPointsToKeep);
}

