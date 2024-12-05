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

bool itsValid(float r, float g, float b)
{
	if(r == 0.0 &&  g == 0.0 && b == 1.0)
		return false;

	return true;
}

void PointDistanceMapperModule::inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose)
{
	typedef Nabo::NearestNeighbourSearch<float> NNS;

	PM::Matches matches(PM::Matches::Dists(1, input.getNbPoints()), PM::Matches::Ids(1, input.getNbPoints()));
	std::shared_ptr<NNS> nns = std::shared_ptr<NNS>(NNS::create(map.features, map.features.rows() - 1,
																NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));

	//knn(const Vector& query, IndexVector& indices, Vector& dists2, const Index k, const T epsilon, const unsigned optionFlags, const T maxRadius)
	nns->knn(input.features, matches.ids, matches.dists, 1, 0);

	int inputPointsToKeepPointCount = 0;
	PM::DataPoints inputPointsToKeep(input.createSimilarEmpty());

	float powMinDistNewPoint = std::pow(minDistNewPoint, 2);
	float colorMinDistNewPoint = powMinDistNewPoint/1000;

	for(int i = 0; i < input.getNbPoints(); ++i)
	{
		float inputR = input.descriptors(0, i);
		float inputG = input.descriptors(1, i);
		float inputB = input.descriptors(2, i);

		float mapR = map.descriptors(0, matches.ids(i));
		float mapG = map.descriptors(1, matches.ids(i));
		float mapB = map.descriptors(2, matches.ids(i));

		float outputR = mapR;
		float outputG = mapG;
		float outputB = mapB;

		bool validInput = itsValid(inputR, inputG, inputB); 
		bool validMap   = itsValid(mapR  , mapG  , mapB  );

		//coloring point
		if(validInput)
		{
			if(validMap)
			{
				if (mapR > inputR)
				{
					outputR = 0.2*inputR+0.8*mapR;
					outputG = 0.2*inputG+0.8*mapG;
					outputB = 0.2*inputB+0.8*mapB;

				}
				else
				{
					outputR = 0.8*inputR+0.2*mapR;
					outputG = 0.8*inputG+0.2*mapG;
					outputB = 0.8*inputB+0.2*mapB;
				}
			}
			else
			{
				outputR = inputR;
				outputG = inputG;
				outputB = inputB;
			}
		}
			
		map.descriptors(0, matches.ids(i)) = outputR;
		map.descriptors(1, matches.ids(i)) = outputG;
		map.descriptors(2, matches.ids(i)) = outputB;
	
		if(matches.dists(i) >= powMinDistNewPoint)
		{
				inputPointsToKeep.setColFrom(inputPointsToKeepPointCount, input, i);
				inputPointsToKeepPointCount++;
		}
		else if(inputR > 0.3 && matches.dists(i) >= colorMinDistNewPoint) //ROI - save denser pointcloud
		{
			inputPointsToKeep.setColFrom(inputPointsToKeepPointCount, input, i);
			inputPointsToKeepPointCount++;
		}
			
	}
	inputPointsToKeep.conservativeResize(inputPointsToKeepPointCount);
    map.concatenate(inputPointsToKeep);
}

