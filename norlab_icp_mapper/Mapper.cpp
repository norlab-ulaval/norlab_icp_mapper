#include "Mapper.h"
#include <fstream>
#include <chrono>

norlab_icp_mapper::Mapper::Mapper(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath,
								  const std::string& mapPostFiltersConfigFilePath, const std::string& mapUpdateCondition, const float& mapUpdateOverlap,
								  const float& mapUpdateDelay, const float& mapUpdateDistance, const float& minDistNewPoint, const float& sensorMaxRange,
								  const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle, const float& epsilonA,
								  const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& isOnline,
								  const bool& computeProbDynamic, const bool& isMapping, const char& saveMapCellsOnHardDrive):
		mapUpdateCondition(mapUpdateCondition),
		mapUpdateOverlap(mapUpdateOverlap),
		mapUpdateDelay(mapUpdateDelay),
		mapUpdateDistance(mapUpdateDistance),
		is3D(is3D),
		isOnline(isOnline),
		isMapping(isMapping),
		map(minDistNewPoint, sensorMaxRange, priorDynamic, thresholdDynamic, beamHalfAngle, epsilonA, epsilonD, alpha, beta, is3D,
			isOnline, computeProbDynamic, saveMapCellsOnHardDrive, icp, icpMapLock),
		trajectory(is3D ? 3 : 2),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation")),
    success_frames(0),
    failed_frames(0)
{
	loadYamlConfig(inputFiltersConfigFilePath, icpConfigFilePath, mapPostFiltersConfigFilePath);

	PM::Parameters radiusFilterParams;
	radiusFilterParams["dim"] = "-1";
	radiusFilterParams["dist"] = std::to_string(sensorMaxRange);
	radiusFilterParams["removeInside"] = "0";
	radiusFilter = PM::get().DataPointsFilterRegistrar.create("DistanceLimitDataPointsFilter", radiusFilterParams);
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

norlab_icp_mapper::DiagnosticInformation norlab_icp_mapper::Mapper::processInput(const PM::DataPoints& inputInSensorFrame,
                                                                                 const PM::TransformationParameters& estimatedPose,
                                                                                 const std::chrono::time_point<std::chrono::steady_clock>& timeStamp)
{
	PM::DataPoints filteredInputInSensorFrameCopy;
	processInput(inputInSensorFrame, estimatedPose, timeStamp,false, filteredInputInSensorFrameCopy);

}

norlab_icp_mapper::DiagnosticInformation norlab_icp_mapper::Mapper::processInput(const PM::DataPoints& inputInSensorFrame,
																																								 const PM::TransformationParameters& estimatedPose,
																																								 const std::chrono::time_point<std::chrono::steady_clock>& timeStamp,
																																								 bool skip_icp)
{
	PM::DataPoints filteredInputInSensorFrameCopy;
	processInput(inputInSensorFrame, estimatedPose, timeStamp,skip_icp, filteredInputInSensorFrameCopy);
}

norlab_icp_mapper::DiagnosticInformation norlab_icp_mapper::Mapper::processInput(const PM::DataPoints& inputInSensorFrame,
																																								 const PM::TransformationParameters& estimatedPose,
																																								 const std::chrono::time_point<std::chrono::steady_clock>& timeStamp,
																																								 bool skip_icp,
																																								 PM::DataPoints& filteredInputInSensorFrameCopy)
{
	std::chrono::time_point<std::chrono::steady_clock> processingStartTime = std::chrono::steady_clock::now();
	DiagnosticInformation info;
	info.nbPointsInputBeforeFiltering = inputInSensorFrame.getNbPoints();

	PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);
	inputFilters.apply(filteredInputInSensorFrame);
	filteredInputInSensorFrameCopy = filteredInputInSensorFrame;

	info.inputFilteringTime = std::chrono::duration<float>(std::chrono::steady_clock::now() - processingStartTime).count();
	info.nbPointsInputAfterFiltering = filteredInputInSensorFrame.getNbPoints();
	info.nbPointsReference = map.getNbPointsLocalPointCloud();

	PM::DataPoints input = transformation->compute(filteredInputInSensorFrame, estimatedPose);

	PM::TransformationParameters correctedPose;
	if(map.isLocalPointCloudEmpty())
	{
		info.estimatedOverlap = 0.0;

		correctedPose = estimatedPose;

		map.updatePose(correctedPose);

		updateMap(input, correctedPose, timeStamp);
	}
	else
	{
		float estimatedOverlap = 0.0;
		PM::TransformationParameters correction = PM::TransformationParameters::Identity(estimatedPose.rows(),
																						 estimatedPose.cols());

		if(!skip_icp) {
			{
        try
        {
				  std::lock_guard<std::mutex> icpMapLockGuard(icpMapLock);
				  correction = icp(input);
          success_frames++;
        }
        catch (const PM::ConvergenceError& convergenceError)
        {
          correction.setIdentity();
          failed_frames++;
        }
      }
			estimatedOverlap = icp.errorMinimizer->getOverlap();
		}

		correctedPose = correction * estimatedPose;
		info.estimatedOverlap = estimatedOverlap;

		map.updatePose(correctedPose);

		if(shouldUpdateMap(timeStamp, correctedPose, estimatedOverlap))
		{
			updateMap(transformation->compute(input, correction), correctedPose, timeStamp);
		}
	}

	poseLock.lock();
	pose = correctedPose;
	poseLock.unlock();

	int euclideanDim = is3D ? 3 : 2;
	trajectoryLock.lock();
	trajectory.addPoint(correctedPose.topRightCorner(euclideanDim, 1));
	trajectoryLock.unlock();

	float processingTime = std::chrono::duration<float>(std::chrono::steady_clock::now() - processingStartTime).count();;
	info.processingTime = processingTime;
	if(lastTimeStamp.time_since_epoch().count() != 0)
	{
		info.processingTimePercentage = 100.0f * processingTime / std::chrono::duration<float>(timeStamp - lastTimeStamp).count();
	}
	lastTimeStamp = timeStamp;
  info.successfully_processed_frames = success_frames;
  info.failed_frames = failed_frames;

	return info;
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
										  const std::chrono::time_point<std::chrono::steady_clock>& currentTimeStamp)
{
	lastTimeMapWasUpdated = currentTimeStamp;
	lastPoseWhereMapWasUpdated = currentPose;

	if(isOnline && !map.isLocalPointCloudEmpty())
	{
		mapUpdateFuture = std::async(std::launch::async, &Map::updateLocalPointCloud, &map, currentInput, currentPose, mapPostFilters);
	}
	else
	{
		map.updateLocalPointCloud(currentInput, currentPose, mapPostFilters);
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

void norlab_icp_mapper::Mapper::clearMap()
{
	map.clearGlobalPointCloud();
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
