#include "Mapper.h"
#include <fstream>
#include <chrono>

norlab_icp_mapper::Mapper::Mapper(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath,
								  const std::string& mapPostFiltersConfigFilePath, const std::string& mapUpdateCondition, const float& mapUpdateOverlap,
								  const float& mapUpdateDelay, const float& mapUpdateDistance, const float& minDistNewPoint, const float& sensorMaxRange,
								  const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle, const float& epsilonA,
								  const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& isOnline,
								  const bool& computeProbDynamic, const bool& isMapping, const bool& saveMapCellsOnHardDrive, const std::string& csvFileName):
		mapUpdateCondition(mapUpdateCondition),
		mapUpdateOverlap(mapUpdateOverlap),
		mapUpdateDelay(mapUpdateDelay),
		mapUpdateDistance(mapUpdateDistance),
		is3D(is3D),
		isOnline(isOnline),
		isMapping(isMapping),
		csvFileName(csvFileName),
		map(minDistNewPoint, sensorMaxRange, priorDynamic, thresholdDynamic, beamHalfAngle, epsilonA, epsilonD, alpha, beta, is3D,
			isOnline, computeProbDynamic, saveMapCellsOnHardDrive, icp, icpMapLock, matcher, matcherLock, csvFileName),
		trajectory(is3D ? 3 : 2),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
{
	loadYamlConfig(inputFiltersConfigFilePath, icpConfigFilePath, mapPostFiltersConfigFilePath);

	PM::Parameters radiusFilterParams;
	radiusFilterParams["dim"] = "-1";
	radiusFilterParams["dist"] = std::to_string(sensorMaxRange);
	radiusFilterParams["removeInside"] = "0";
	radiusFilter = PM::get().DataPointsFilterRegistrar.create("DistanceLimitDataPointsFilter", radiusFilterParams);

	initializeCSVFile(csvFileName);

	PM::Parameters matcherParams;
	matcherParams["knn"] = "1";
	matcherParams["maxDist"] = "1";
	matcherParams["epsilon"] = "0";
	matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", matcherParams);

	PM::Parameters vicinityFilterParams;
	vicinityFilterParams["dim"] = "-1";
	vicinityFilterParams["dist"] = std::to_string(3.0);
	vicinityFilterParams["removeInside"] = "0";
	vicinityFilter = PM::get().DataPointsFilterRegistrar.create("DistanceLimitDataPointsFilter", vicinityFilterParams);
}

void norlab_icp_mapper::Mapper::loadYamlConfig(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath,
											   const std::string& mapPostFiltersConfigFilePath)
{
	if(!icpConfigFilePath.empty())
	{
		std::ifstream ifs(icpConfigFilePath.c_str());
		icp.loadFromYaml(ifs);
		ifs.close();
	}
	else
	{
		icp.setDefault();
	}

	if(!inputFiltersConfigFilePath.empty())
	{
		std::ifstream ifs(inputFiltersConfigFilePath.c_str());
		inputFilters = PM::DataPointsFilters(ifs);
		ifs.close();
	}

	if(!mapPostFiltersConfigFilePath.empty())
	{
		std::ifstream ifs(mapPostFiltersConfigFilePath.c_str());
		mapPostFilters = PM::DataPointsFilters(ifs);
		ifs.close();
	}
}

void norlab_icp_mapper::Mapper::processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& estimatedPose,
											 const std::chrono::time_point<std::chrono::steady_clock>& timeStamp)
{
	PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);
	inputFilters.apply(filteredInputInSensorFrame);
	PM::DataPoints input = transformation->compute(filteredInputInSensorFrame, estimatedPose);

	PM::TransformationParameters correctedPose;
	if(map.isLocalPointCloudEmpty())
	{
		correctedPose = estimatedPose;

		map.updatePose(correctedPose);

		updateMap(input, correctedPose, timeStamp, CSVLine());
	}
	else
	{
		icpMapLock.lock();
		PM::TransformationParameters correction = icp(input);
		icpMapLock.unlock();

		correctedPose = correction * estimatedPose;

		PM::DataPoints correctedInput = transformation->compute(input, correction);
		PM::DataPoints localPointCloud = map.getLocalPointCloud();
		matcherLock.lock();
		PM::Matches matches(matcher->findClosests(correctedInput));
		matcherLock.unlock();

		Quaternion quaternion = convertRotationMatrixToQuaternion(correctedPose.topLeftCorner<3, 3>());
		float meanResidual = computeMeanResidual(correctedInput, localPointCloud, matches);
		int nbPointsVicinity = computeNbPointsVicinity(filteredInputInSensorFrame);

		CSVLine csvLine = {timeStamp, correctedPose(0, 3), correctedPose(1, 3), correctedPose(2, 3), quaternion.w,
						   quaternion.x, quaternion.y, quaternion.z, meanResidual, correction, -1, nbPointsVicinity};
		map.updatePose(correctedPose);

		if(!shouldUpdateMap(timeStamp, correctedPose, icp.errorMinimizer->getOverlap()))
		{
			logToCSV(csvLine, csvFileName);
		}
		else
		{
			updateMap(correctedInput, correctedPose, timeStamp, csvLine);
		}
	}

	poseLock.lock();
	pose = correctedPose;
	poseLock.unlock();

	int euclideanDim = is3D ? 3 : 2;
	trajectoryLock.lock();
	trajectory.addPoint(correctedPose.topRightCorner(euclideanDim, 1));
	trajectoryLock.unlock();
}

Quaternion norlab_icp_mapper::Mapper::convertRotationMatrixToQuaternion(const PM::TransformationParameters& matrix) const
{
	Quaternion q;

	float trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);
	if(trace > 0)
	{
		float s = 0.5f / std::sqrt(trace + 1.0f);
		q.w = 0.25f / s;
		q.x = (matrix(2, 1) - matrix(1, 2)) * s;
		q.y = (matrix(0, 2) - matrix(2, 0)) * s;
		q.z = (matrix(1, 0) - matrix(0, 1)) * s;
	}
	else
	{
		if(matrix(0, 0) > matrix(1, 1) && matrix(0, 0) > matrix(2, 2))
		{
			float s = 2.0f * std::sqrt(1.0f + matrix(0, 0) - matrix(1, 1) - matrix(2, 2));
			q.w = (matrix(2, 1) - matrix(1, 2)) / s;
			q.x = 0.25f * s;
			q.y = (matrix(0, 1) + matrix(1, 0)) / s;
			q.z = (matrix(0, 2) + matrix(2, 0)) / s;
		}
		else if(matrix(1, 1) > matrix(2, 2))
		{
			float s = 2.0f * std::sqrt(1.0f + matrix(1, 1) - matrix(0, 0) - matrix(2, 2));
			q.w = (matrix(0, 2) - matrix(2, 0)) / s;
			q.x = (matrix(0, 1) + matrix(1, 0)) / s;
			q.y = 0.25f * s;
			q.z = (matrix(1, 2) + matrix(2, 1)) / s;
		}
		else
		{
			float s = 2.0f * std::sqrt(1.0f + matrix(2, 2) - matrix(0, 0) - matrix(1, 1));
			q.w = (matrix(1, 0) - matrix(0, 1)) / s;
			q.x = (matrix(0, 2) + matrix(2, 0)) / s;
			q.y = (matrix(1, 2) + matrix(2, 1)) / s;
			q.z = 0.25f * s;
		}
	}

	return q;
}

float norlab_icp_mapper::Mapper::computeMeanResidual(const PM::DataPoints& correctedInput, const PM::DataPoints& localPointCloud, const PM::Matches& matches)
const
{
	double error = 0;
	int nbValidPoints = 0;
	for(int i = 0; i < matches.ids.cols(); i++)
	{
		Eigen::Vector3f inputPoint = correctedInput.features.col(i).head(3);
		Eigen::Vector3f mapPoint = localPointCloud.features.col(matches.ids(0, i)).head(3);
		Eigen::Vector3f normal = localPointCloud.getDescriptorViewByName("normals").col(matches.ids(0, i));
		float normalNorm = normal.norm();
		if(normalNorm > 0)
		{
			normal /= normalNorm;
			error += std::fabs((mapPoint - inputPoint).dot(normal));
			nbValidPoints++;
		}
	}
	return (float)(error / (double)nbValidPoints);
}

int norlab_icp_mapper::Mapper::computeNbPointsVicinity(const PM::DataPoints& filteredInputInSensorFrame) const
{
	return vicinityFilter->filter(filteredInputInSensorFrame).getNbPoints();
}

bool norlab_icp_mapper::Mapper::shouldUpdateMap(const std::chrono::time_point<std::chrono::steady_clock>& currentTime,
												const PM::TransformationParameters& currentPose, const float& currentOverlap) const
{
	if(!isMapping.load())
	{
		return false;
	}

	if(isOnline)
	{
		// if previous update is not over
		if(mapUpdateFuture.valid() && mapUpdateFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
		{
			return false;
		}
	}

	if(mapUpdateCondition == "overlap")
	{
		return currentOverlap < mapUpdateOverlap;
	}
	else if(mapUpdateCondition == "delay")
	{
		return (currentTime - lastTimeMapWasUpdated) > std::chrono::duration<float>(mapUpdateDelay);
	}
	else
	{
		int euclideanDim = is3D ? 3 : 2;
		PM::Vector lastLocation = lastPoseWhereMapWasUpdated.topRightCorner(euclideanDim, 1);
		PM::Vector currentLocation = currentPose.topRightCorner(euclideanDim, 1);
		return std::fabs((currentLocation - lastLocation).norm()) > mapUpdateDistance;
	}
}

void norlab_icp_mapper::Mapper::updateMap(const PM::DataPoints& currentInput, const PM::TransformationParameters& currentPose,
										  const std::chrono::time_point<std::chrono::steady_clock>& currentTimeStamp, const CSVLine& csvLine)
{
	lastTimeMapWasUpdated = currentTimeStamp;
	lastPoseWhereMapWasUpdated = currentPose;

	if(isOnline && !map.isLocalPointCloudEmpty())
	{
		mapUpdateFuture = std::async(std::launch::async, &Map::updateLocalPointCloud, &map, currentInput, currentPose, mapPostFilters, csvLine);
	}
	else
	{
		map.updateLocalPointCloud(currentInput, currentPose, mapPostFilters, csvLine);
	}
}

norlab_icp_mapper::Mapper::PM::DataPoints norlab_icp_mapper::Mapper::getMap()
{
	return map.getGlobalPointCloud();
}

void norlab_icp_mapper::Mapper::setMap(const PM::DataPoints& newMap)
{
	map.setGlobalPointCloud(newMap);
	trajectoryLock.lock();
	trajectory.clearPoints();
	trajectoryLock.unlock();
}

bool norlab_icp_mapper::Mapper::getNewLocalMap(PM::DataPoints& mapOut)
{
	return map.getNewLocalPointCloud(mapOut);
}

norlab_icp_mapper::Mapper::PM::TransformationParameters norlab_icp_mapper::Mapper::getPose()
{
	std::lock_guard<std::mutex> lock(poseLock);
	return pose;
}

bool norlab_icp_mapper::Mapper::getIsMapping() const
{
	return isMapping.load();
}

void norlab_icp_mapper::Mapper::setIsMapping(const bool& newIsMapping)
{
	isMapping.store(newIsMapping);
}

Trajectory norlab_icp_mapper::Mapper::getTrajectory()
{
	std::lock_guard<std::mutex> lock(trajectoryLock);
	return trajectory;
}
