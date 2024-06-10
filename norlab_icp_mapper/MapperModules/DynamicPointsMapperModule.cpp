#include <nabo/nabo.h>
#include "DynamicPointsMapperModule.h"

DynamicPointsMapperModule::DynamicPointsMapperModule(const PM::Parameters& params):
	MapperModule("DynamicPointsMapperModule", DynamicPointsMapperModule::availableParameters(), params),
	    priorDynamic(PM::Parametrizable::get<float>("priorDynamic")),
	    thresholdDynamic(PM::Parametrizable::get<float>("thresholdDynamic")),
	    alpha(PM::Parametrizable::get<float>("alpha")),
	    beta(PM::Parametrizable::get<float>("beta")),
	    beamHalfAngle(PM::Parametrizable::get<float>("beamHalfAngle")),
	    epsilonA(PM::Parametrizable::get<float>("epsilonA")),
		epsilonD(PM::Parametrizable::get<float>("epsilonD")),
		sensorMaxRange(PM::Parametrizable::get<float>("sensorMaxRange")),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
    {}

PointMatcher<float>::DataPoints DynamicPointsMapperModule::createMap(const PM::DataPoints& input, const PM::TransformationParameters& pose)
{
    DataPoints outputMap(input);
    inPlaceCreateMap(outputMap, pose);
    return outputMap;
}

void DynamicPointsMapperModule::inPlaceCreateMap(PM::DataPoints& input, const PM::TransformationParameters& pose)
{
}

PointMatcher<float>::DataPoints DynamicPointsMapperModule::updateMap(const PM::DataPoints& input, const PM::DataPoints& map, const PM::TransformationParameters& pose)
{
    DataPoints outputMap(map);
    inPlaceUpdateMap(input, outputMap, pose);
    return outputMap;
}

void DynamicPointsMapperModule::inPlaceUpdateMap(const PM::DataPoints& input, PM::DataPoints& map, const PM::TransformationParameters& pose)
{
    // assume the input already has the probabilityDynamic descriptor
    // can be added in yaml config with the AddDescriptorDataPointsFilter
    if(! input.descriptorExists("probabilityDynamic"))
        throw PM::DataPoints::InvalidField("Missing field 'probabilityDynamic' in input point cloud");
    typedef Nabo::NearestNeighbourSearch<float> NNS;
	const float eps = 0.0001;

	PM::DataPoints inputInSensorFrame = transformation->compute(input, pose.inverse());

	PM::Matrix inputInSensorFrameRadii;
	PM::Matrix inputInSensorFrameAngles;
	convertToSphericalCoordinates(inputInSensorFrame, inputInSensorFrameRadii, inputInSensorFrameAngles);

	PM::DataPoints currentLocalPointCloudInSensorFrame = transformation->compute(map, pose.inverse());
	PM::Matrix globalId(1, map.getNbPoints());
	int nbPointsWithinSensorMaxRange = 0;
	for(int i = 0; i < map.getNbPoints(); i++)
	{
		if(currentLocalPointCloudInSensorFrame.features.col(i).head(currentLocalPointCloudInSensorFrame.getEuclideanDim()).norm() < sensorMaxRange)
		{
			currentLocalPointCloudInSensorFrame.setColFrom(nbPointsWithinSensorMaxRange, currentLocalPointCloudInSensorFrame, i);
			globalId(0, nbPointsWithinSensorMaxRange) = i;
			nbPointsWithinSensorMaxRange++;
		}
	}
	currentLocalPointCloudInSensorFrame.conservativeResize(nbPointsWithinSensorMaxRange);

	PM::Matrix currentLocalPointCloudInSensorFrameRadii;
	PM::Matrix currentLocalPointCloudInSensorFrameAngles;
	convertToSphericalCoordinates(currentLocalPointCloudInSensorFrame, currentLocalPointCloudInSensorFrameRadii, currentLocalPointCloudInSensorFrameAngles);

	std::shared_ptr<NNS> nns = std::shared_ptr<NNS>(NNS::create(inputInSensorFrameAngles));
	PM::Matches::Dists dists(1, currentLocalPointCloudInSensorFrame.getNbPoints());
	PM::Matches::Ids ids(1, currentLocalPointCloudInSensorFrame.getNbPoints());
	nns->knn(currentLocalPointCloudInSensorFrameAngles, ids, dists, 1, 0, NNS::ALLOW_SELF_MATCH, 2 * beamHalfAngle);

	PM::DataPoints::View viewOnProbabilityDynamic = map.getDescriptorViewByName("probabilityDynamic");
	PM::DataPoints::View viewOnNormals = currentLocalPointCloudInSensorFrame.getDescriptorViewByName("normals");
	for(int i = 0; i < currentLocalPointCloudInSensorFrame.getNbPoints(); i++)
	{
		if(dists(i) != std::numeric_limits<float>::infinity())
		{
			const int inputPointId = ids(0, i);
			const int localPointCloudPointId = globalId(0, i);

			const Eigen::VectorXf inputPoint = inputInSensorFrame.features.col(inputPointId).head(inputInSensorFrame.getEuclideanDim());
			const Eigen::VectorXf
					localPointCloudPoint = currentLocalPointCloudInSensorFrame.features.col(i).head(currentLocalPointCloudInSensorFrame.getEuclideanDim());
			const float delta = (inputPoint - localPointCloudPoint).norm();
			const float d_max = epsilonA * inputPoint.norm();

			const Eigen::VectorXf localPointCloudPointNormal = viewOnNormals.col(i);

			const float w_v = eps + (1. - eps) * fabs(localPointCloudPointNormal.dot(localPointCloudPoint.normalized()));
			const float w_d1 = eps + (1. - eps) * (1. - sqrt(dists(i)) / (2 * beamHalfAngle));

			const float offset = delta - epsilonD;
			float w_d2 = 1.;
			if(delta < epsilonD || localPointCloudPoint.norm() > inputPoint.norm())
			{
				w_d2 = eps;
			}
			else
			{
				if(offset < d_max)
				{
					w_d2 = eps + (1 - eps) * offset / d_max;
				}
			}

			float w_p2 = eps;
			if(delta < epsilonD)
			{
				w_p2 = 1;
			}
			else
			{
				if(offset < d_max)
				{
					w_p2 = eps + (1. - eps) * (1. - offset / d_max);
				}
			}

			if((inputPoint.norm() + epsilonD + d_max) >= localPointCloudPoint.norm())
			{
				const float lastDyn = viewOnProbabilityDynamic(0, localPointCloudPointId);

				const float c1 = (1 - (w_v * w_d1));
				const float c2 = w_v * w_d1;

				float probDynamic;
				float probStatic;
				if(lastDyn < thresholdDynamic)
				{
					probDynamic = c1 * lastDyn + c2 * w_d2 * ((1 - alpha) * (1 - lastDyn) + beta * lastDyn);
					probStatic = c1 * (1 - lastDyn) + c2 * w_p2 * (alpha * (1 - lastDyn) + (1 - beta) * lastDyn);
				}
				else
				{
					probDynamic = 1 - eps;
					probStatic = eps;
				}

				viewOnProbabilityDynamic(0, localPointCloudPointId) = probDynamic / (probDynamic + probStatic);
			}
		}
	}
}




void DynamicPointsMapperModule::convertToSphericalCoordinates(const PM::DataPoints& points, PM::Matrix& radii, PM::Matrix& angles) const
{
    bool is3D = points.features.rows() - 1 == 3;
	radii = points.features.topRows(points.getEuclideanDim()).colwise().norm();
	angles = PM::Matrix(2, points.getNbPoints());

	for(int i = 0; i < points.getNbPoints(); i++)
	{
		angles(0, i) = 0;
		if(is3D)
		{
			const float ratio = points.features(2, i) / radii(0, i);
			angles(0, i) = asin(ratio);
		}
		angles(1, i) = atan2(points.features(1, i), points.features(0, i));
	}
}

