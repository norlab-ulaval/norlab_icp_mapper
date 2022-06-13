#include "Mapper.h"
#include <fstream>
#include <chrono>

norlab_icp_mapper::Mapper::Mapper(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath,
								  const std::string& mapPostFiltersConfigFilePath, const std::string& mapUpdateCondition, const float& mapUpdateOverlap,
								  const float& mapUpdateDelay, const float& mapUpdateDistance, const float& minDistNewPoint, const float& sensorMaxRange,
								  const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle, const float& epsilonA,
								  const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& isOnline,
								  const bool& computeProbDynamic, const bool& isMapping, const bool& saveMapCellsOnHardDrive):
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
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
{
	loadYamlConfig(inputFiltersConfigFilePath, icpConfigFilePath, mapPostFiltersConfigFilePath);

	std::string sampingParamsFileName = "/tmp/map_sampling_params.txt";
	std::fstream sampingParamsFile;
	sampingParamsFile.open(sampingParamsFileName,std::ios::in); //open a file to perform read operation using file object
	if (sampingParamsFile.is_open()){ //checking whether the file is open
		std::string tmp;
		getline(sampingParamsFile, filterName);
		getline(sampingParamsFile, tmp);
		compressionRatio = std::stof(tmp);
		getline(sampingParamsFile, tmp);
		desiredCompressionRatio = std::stof(tmp);
		getline(sampingParamsFile, tmp);
		std::istringstream("1") >> removeWall;
		std::cout << "Open file " << sampingParamsFileName << "\n"
					<< "Loaded name: " << filterName << ", desired CR: " << desiredCompressionRatio << ", init CR: " << compressionRatio << std::endl;
		sampingParamsFile.close(); //close the file object.
		}
	else
	{
		std::cout << "Could not open the file: " << sampingParamsFileName << std::endl;
		throw std::exception();
	}


	if(desiredCompressionRatio != 0.0)
	{
		std::string lookupTableFilePath =
			"/home/mbo/norlab_ws/playground/norlab_utils/data/sampling_lookup_tables/map_" + filterName + ".csv";
		std::fstream lookupTableFile;
		lookupTableFile.open(lookupTableFilePath, std::ios::in); //open a file to perform read operation using file object
		std::vector<std::string> row;
		std::string line, word;
		if (lookupTableFile.is_open())
		{
//		skip header line
			getline(lookupTableFile, line);
			while (getline(lookupTableFile, line))
			{
				row.clear();

				std::stringstream str(line);

				std::getline(str, word, ',');
				std::getline(str, word, ',');
				compRatios.push_back(std::stof(word));
				std::getline(str, word, ',');
				paramValues.push_back(std::stof(word));
			}
			lookupTableFile.close();
		}
		else
			std::cout << "Could not open the file: " << lookupTableFilePath << std::endl;
	}


//	print to log file
	std::freopen("/tmp/mapper_logger.txt", "w", stderr);
	std::cerr << "nbPointsScan,nbPointsScanInputFilters,comulativeNbPointsScan,"
				 "nbPointsOriginalMap,nbPointsMapIcpFilters,nbPointsMapPostFilters,"
				 "icpUpdateDuration,mapUpdateDuration" << std::endl;
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

void norlab_icp_mapper::Mapper::processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& estimatedPose,
											 const std::chrono::time_point<std::chrono::steady_clock>& timeStamp)
{
	unsigned int nbPointsScan;
	unsigned int nbPointsScanInputFilters;
	unsigned int nbPointsOriginalMap = map.getLocalPointCloud().getNbPoints();
	int nbPointsMapIcpFilters = -1;
	unsigned int nbPointsMapPostFilters;
	long icpUpdateDuration = -1;
	mapUpdateDuration = -1;

	PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);
	nbPointsScan = filteredInputInSensorFrame.getNbPoints();

	inputFilters.apply(filteredInputInSensorFrame);
	PM::DataPoints input = transformation->compute(filteredInputInSensorFrame, estimatedPose);

	nbPointsScanInputFilters = input.getNbPoints();
	PM::TransformationParameters correctedPose;
	if(map.isLocalPointCloudEmpty())
	{
		correctedPose = estimatedPose;

		map.updatePose(correctedPose);

		auto start = std::chrono::high_resolution_clock::now();
		comulativeNbScanPoints += input.getNbPoints();
		updateMap(input, correctedPose, timeStamp);
		auto stop = std::chrono::high_resolution_clock::now();
		mapUpdateDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	}
	else
	{
		auto start = std::chrono::high_resolution_clock::now();
		PM::TransformationParameters correction;
		{
			std::lock_guard<std::mutex> icpMapLockGuard(icpMapLock);
			correction = icp(input);
		}
		auto stop = std::chrono::high_resolution_clock::now();
		icpUpdateDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
		nbPointsMapIcpFilters = icp.getPrefilteredMap().getNbPoints();

		correctedPose = correction * estimatedPose;

		map.updatePose(correctedPose);

		if(shouldUpdateMap(timeStamp, correctedPose, icp.errorMinimizer->getOverlap()))
		{
			comulativeNbScanPoints += input.getNbPoints();
			updateMap(transformation->compute(input, correction), correctedPose, timeStamp);
		}
	}
	nbPointsMapPostFilters = map.getLocalPointCloud().getNbPoints();

	std::clog << nbPointsScan << ","
			  << nbPointsScanInputFilters << ","
			  << comulativeNbScanPoints << ","
			<< nbPointsOriginalMap << ","
			<< nbPointsMapIcpFilters << ","
			<< nbPointsMapPostFilters << ","
			<< icpUpdateDuration << ","
			<< mapUpdateDuration << std::endl;

	poseLock.lock();
	pose = correctedPose;
	poseLock.unlock();

	int euclideanDim = is3D ? 3 : 2;
	trajectoryLock.lock();
	trajectory.addPoint(correctedPose.topRightCorner(euclideanDim, 1));
	trajectoryLock.unlock();
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


float getClosestParamForCR(const std::vector<float> &compRatios, const std::vector<float> &paramValues, float compressionRatio)
{
	auto iter_closest = std::lower_bound(compRatios.begin(), compRatios.end(), 100.0 * compressionRatio);
	auto idx_closest = iter_closest - compRatios.begin();
	if(idx_closest == compRatios.size())
		idx_closest -= 1;
	float param_value = paramValues[idx_closest];
//	std::cout << "For CR: " << compressionRatio <<", Closest: idx: " << idx_closest << " cr value: " << compRatios[idx_closest] << " param: "
//			  << param_value << std::endl;
	return param_value;
}

float getClosestCRForParamValue(const std::vector<float> &compRatios, const std::vector<float> &paramValues, float paramValue)
{
//	reversed order
	size_t idx_closest;
	if (paramValues[0] > paramValues[paramValues.size()-1])
	{
		auto iter_closest = std::lower_bound(paramValues.begin(), paramValues.end(), paramValue, std::greater<float>());
		idx_closest = iter_closest - paramValues.begin();
	}
	else
	{
		auto iter_closest = std::lower_bound(paramValues.begin(), paramValues.end(), paramValue);
		idx_closest = iter_closest - paramValues.begin();
	}
	if(idx_closest == paramValues.size())
		idx_closest -= 1;
	float cr = compRatios[idx_closest];
//	std::cout << "For param: " << paramValue << ", Closest: idx: " << idx_closest << " found param value: " << paramValues[idx_closest] << " cr: "
//			  << cr << std::endl;
	return cr/100.0;
}


std::shared_ptr<norlab_icp_mapper::Mapper::PM::DataPointsFilter> norlab_icp_mapper::Mapper::getFilter(PM::Parameters filterParams,
	std::shared_ptr<PM::DataPointsFilter> filter, const std::string &paramName, const PM::DataPoints &ptCloud)
{
	bool end = false;
	int maxIter = 100;
	int iter = 0;
	int sign = 1.0;
	std::string paramOldTMinus2 = "dummy2";
	std::string paramOldTMinus1 = "dummy1";
	std::string paramBeforeStr = filterParams[paramName];
	std::string paramAfterStr = filterParams[paramName];
	while (true)
	{
		iter++;
		auto filteredCloud = filter->filter(ptCloud);
		double achievedCR = (1.0 - ((float)filteredCloud.getNbPoints() / (float)comulativeNbScanPoints));

		if (std::abs(desiredCompressionRatio - achievedCR) < (0.5/100.0) || iter >= maxIter || end || paramOldTMinus2 == paramAfterStr) {
			std::cout << "-----" << paramName << ": Before: " << paramBeforeStr << ", After: " << paramAfterStr << ", Iter: " << iter << std::endl;
			if(paramBeforeStr != paramAfterStr)
			{
				compressionRatio = getClosestCRForParamValue(compRatios, paramValues, std::stof(paramAfterStr));
				std::cout << "-----New CR: " << compressionRatio << std::endl;
			}
			std::cout << "--------Leaving get_Filter() function-----------" << std::endl;
			return filter;
		}
		if (desiredCompressionRatio > achievedCR)
		{
			sign = -1;
		}
		else if (desiredCompressionRatio < achievedCR)
		{
			sign = 1;
		}

		if (paramName == "maxDensity")
		{
			float paramBefore = std::stof(filterParams[paramName]);
			float paramAfter = paramBefore + (sign*5.0);
			if (iter == 1)
				paramBeforeStr = filterParams[paramName];
			if(paramAfter < 0.0000001)
			{
				end = true;
				continue;
			}
			filterParams[paramName] = std::to_string(paramAfter);
			paramOldTMinus2 = paramOldTMinus1;
			paramOldTMinus1 = paramAfterStr;
			paramAfterStr = filterParams[paramName];
			filter = PM::get().DataPointsFilterRegistrar.create("MaxDensityDataPointsFilter", filterParams);
		}
		else if (paramName == "maxPointByNode")
		{
			int paramBefore = std::stoi(filterParams[paramName]);
			int paramAfter = paramBefore - (sign*1);
			if (iter == 1)
				paramBeforeStr = filterParams[paramName];
			if(paramAfter < 1)
			{
				end = true;
				continue;
			}
			filterParams[paramName] = std::to_string(static_cast<std::size_t>(paramAfter));
			paramOldTMinus2 = paramOldTMinus1;
			paramOldTMinus1 = paramAfterStr;
			paramAfterStr = filterParams[paramName];
			filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
		}
		else if (paramName == "ratio")
		{
			float paramBefore = std::stof(filterParams[paramName]);
			float paramAfter = paramBefore + (sign*0.025);
			if (iter == 1)
				paramBeforeStr = filterParams[paramName];
			if(paramAfter < 0.0000001 || paramAfter > 0.9999999)
			{
				end = true;
				continue;
			}
			filterParams[paramName] = std::to_string(paramAfter);
			paramOldTMinus2 = paramOldTMinus1;
			paramOldTMinus1 = paramAfterStr;
			paramAfterStr = filterParams[paramName];
			filter = PM::get().DataPointsFilterRegistrar.create("SamplingSurfaceNormalDataPointsFilter", filterParams);
		}
		else if (paramName == "radius")
		{
			float paramBefore = std::stof(filterParams[paramName]);
			float paramAfter = paramBefore - (sign*0.01);
			if (iter == 1)
				paramBeforeStr = filterParams[paramName];
			if(paramAfter < 0.0)
			{
				end = true;
				continue;
			}
			filterParams[paramName] = std::to_string(paramAfter);
			paramOldTMinus2 = paramOldTMinus1;
			paramOldTMinus1 = paramAfterStr;
			paramAfterStr = filterParams[paramName];
			filter = PM::get().DataPointsFilterRegistrar.create("SpectralDecompositionDataPointsFilter", filterParams);
		}
		else {
			std::cout << "unknown param name" << std::endl;
		}
	}
}


norlab_icp_mapper::Mapper::PM::DataPoints norlab_icp_mapper::Mapper::getOptimallyFilteredCloud(PM::Parameters filterParams,
	std::shared_ptr<PM::DataPointsFilter> filter, const std::string &paramName, const PM::DataPoints &ptCloud)
{
	bool end = false;
	int maxIter = 100;
	int iter = 0;
	int sign = 1.0;
	long tmpUpdateDuration = 0;
	std::string paramOldTMinus1 = "dummy1";
	std::string paramOldTMinus2 = "dummy2";
	PM::DataPoints filteredCloud;
	PM::DataPoints pointCloudOldTMinus1;
	double crOldTMinus1;
	double achievedCR = 0;
	std::string paramBeforeStr = filterParams[paramName];
	std::string paramAfterStr = filterParams[paramName];
	while (true)
	{
		iter++;

		pointCloudOldTMinus1 = filteredCloud;
		auto start = std::chrono::high_resolution_clock::now();
		filteredCloud = filter->filter(ptCloud);
		auto stop = std::chrono::high_resolution_clock::now();
		tmpUpdateDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

		crOldTMinus1 = achievedCR;
		achievedCR = (1.0 - ((float)filteredCloud.getNbPoints() / (float)comulativeNbScanPoints));

		if (std::abs(desiredCompressionRatio - achievedCR) < (0.5/100.0) || iter >= maxIter || end || paramOldTMinus2 == paramAfterStr) {
//			std::cout << "-----" << paramName << ": Before: " << paramBeforeStr << ", After: " << paramAfterStr << ", Iter: " << iter << std::endl;
			if(paramOldTMinus2 == paramAfterStr &&
				std::abs(desiredCompressionRatio - crOldTMinus1) < std::abs(desiredCompressionRatio - achievedCR))
			{
				filteredCloud = pointCloudOldTMinus1;
			}
			if(paramBeforeStr != paramAfterStr)
			{
				compressionRatio = getClosestCRForParamValue(compRatios, paramValues, std::stof(paramAfterStr));
//				std::cout << "-----New CR: " << compressionRatio << std::endl;
			}
//			std::cout << "--------Leaving getOptimallyFilteredCloud() function-----------" << std::endl;
			mapUpdateDuration += tmpUpdateDuration;
			return filteredCloud;
		}
		if (desiredCompressionRatio > achievedCR)
		{
			sign = -1;
		}
		else if (desiredCompressionRatio < achievedCR)
		{
			sign = 1;
		}

		if (paramName == "maxDensity")
		{
			float paramBefore = std::stof(filterParams[paramName]);
			float paramAfter = paramBefore + (sign*5.0);
			if (iter == 1)
				paramBeforeStr = filterParams[paramName];
			if(paramAfter < 0.0000001)
			{
				end = true;
				continue;
			}
			filterParams[paramName] = std::to_string(paramAfter);
			paramOldTMinus2 = paramOldTMinus1;
			paramOldTMinus1 = paramAfterStr;
			paramAfterStr = filterParams[paramName];
			filter = PM::get().DataPointsFilterRegistrar.create("MaxDensityDataPointsFilter", filterParams);
		}
		else if (paramName == "maxPointByNode")
		{
			int paramBefore = std::stoi(filterParams[paramName]);
			int paramAfter = paramBefore - (sign*1);
			if (iter == 1)
				paramBeforeStr = filterParams[paramName];
			if(paramAfter < 1)
			{
				end = true;
				continue;
			}
			filterParams[paramName] = std::to_string(static_cast<std::size_t>(paramAfter));
			paramOldTMinus2 = paramOldTMinus1;
			paramOldTMinus1 = paramAfterStr;
			paramAfterStr = filterParams[paramName];
			filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
		}
		else if (paramName == "ratio")
		{
			float paramBefore = std::stof(filterParams[paramName]);
			float paramAfter = paramBefore + (sign*0.025);
			if (iter == 1)
				paramBeforeStr = filterParams[paramName];
			if(paramAfter < 0.0000001 || paramAfter > 0.9999999)
			{
				end = true;
				continue;
			}
			filterParams[paramName] = std::to_string(paramAfter);
			paramOldTMinus2 = paramOldTMinus1;
			paramOldTMinus1 = paramAfterStr;
			paramAfterStr = filterParams[paramName];
			filter = PM::get().DataPointsFilterRegistrar.create("SamplingSurfaceNormalDataPointsFilter", filterParams);
		}
		else if (paramName == "radius")
		{
			float paramBefore = std::stof(filterParams[paramName]);
			float paramAfter = paramBefore - (sign*0.01);
			if (iter == 1)
				paramBeforeStr = filterParams[paramName];
			if(paramAfter < 0.0)
			{
				end = true;
				continue;
			}
			filterParams[paramName] = std::to_string(paramAfter);
			paramOldTMinus2 = paramOldTMinus1;
			paramOldTMinus1 = paramAfterStr;
			paramAfterStr = filterParams[paramName];
			filter = PM::get().DataPointsFilterRegistrar.create("SpectralDecompositionDataPointsFilter", filterParams);
		}
		else {
			std::cout << "unknown param name" << std::endl;
		}
	}
}

long norlab_icp_mapper::Mapper::updateMap(const PM::DataPoints& currentInput, const PM::TransformationParameters& currentPose,
										  const std::chrono::time_point<std::chrono::steady_clock>& currentTimeStamp)
{
	mapUpdateDuration = 0;
	lastTimeMapWasUpdated = currentTimeStamp;
	lastPoseWhereMapWasUpdated = currentPose;

	if(isOnline && !map.isLocalPointCloudEmpty())
	{
		mapUpdateFuture = std::async(std::launch::async, &Map::updateLocalPointCloud, &map, currentInput, currentPose, mapPostFilters, removeWall);
	}
	else
	{
		auto start = std::chrono::high_resolution_clock::now();
		map.updateLocalPointCloud(currentInput, currentPose, mapPostFilters, removeWall);
		auto stop = std::chrono::high_resolution_clock::now();
		mapUpdateDuration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
		if(desiredCompressionRatio != 0.0) {
			size_t pointsNow = map.getLocalPointCloud().getNbPoints();

			std::string paramName;

			auto pointDesired = (size_t)((1.0 - compressionRatio) * (float)comulativeNbScanPoints);
			float prob = (1.0 - compressionRatio) * ((float)comulativeNbScanPoints / (float)pointsNow);
			float cr = 1.0 - prob;

			PM::Parameters filterParams;
			auto filters = PM::DataPointsFilters();
			std::shared_ptr<PM::DataPointsFilter> filter;
			if (filterName == "normalSpace" || filterName == "covariance")
			{
				std::string tmpName;
				if (filterName == "normalSpace")
					tmpName = "NormalSpaceDataPointsFilter";
				else
					tmpName = "CovarianceSamplingDataPointsFilter";

				auto nbSample = static_cast<std::size_t>((1.0 - compressionRatio) * (float)comulativeNbScanPoints);
				filterParams["nbSample"] = std::to_string(nbSample);
				filter = PM::get().DataPointsFilterRegistrar.create(tmpName, filterParams);
			}
			else
			{
	//			std::cout << "pointsNow: " << pointsNow << " pointDesired: " << pointDesired << " cum: " << comulativeNbScanPoints << " cr: " << cr << " prob: " << prob << std::endl;
				if (filterName == "random")
				{
					filterParams["prob"] = std::to_string(prob);
					filter = PM::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", filterParams);
					filters.push_back(filter);
					start = std::chrono::high_resolution_clock::now();
					map.applyPostFilters(currentPose, filters);
					stop = std::chrono::high_resolution_clock::now();
				}
				else
				{
					float param_value = getClosestParamForCR(compRatios, paramValues, compressionRatio);

					if (filterName == "maxDensity")
					{
						paramName = "maxDensity";
						filterParams[paramName] = std::to_string(param_value);
						filter = PM::get().DataPointsFilterRegistrar.create("MaxDensityDataPointsFilter", filterParams);
					}
					else if (filterName == "octreeFirstPoint")
					{
						paramName = "maxPointByNode";
						filterParams["samplingMethod"] = "0";
						filterParams[paramName] = std::to_string(static_cast<std::size_t>(param_value));
						filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
					}
					else if (filterName == "octreeRandom")
					{
						paramName = "maxPointByNode";
						filterParams["samplingMethod"] = "1";
						filterParams[paramName] = std::to_string(static_cast<std::size_t>(param_value));
						filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
					}
					else if (filterName == "octreeCentroid")
					{
						paramName = "maxPointByNode";
						filterParams["samplingMethod"] = "2";
						filterParams[paramName] = std::to_string(static_cast<std::size_t>(param_value));
						filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
					}
					else if (filterName == "octreeMedoid")
					{
						paramName = "maxPointByNode";
						filterParams["samplingMethod"] = "3";
						filterParams[paramName] = std::to_string(static_cast<std::size_t>(param_value));
						filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
					}
					else if (filterName == "samplingSurfaceNormal")
					{
						paramName = "ratio";
						filterParams["keepNormals"] = "0";
						filterParams[paramName] = std::to_string(param_value);
						filter = PM::get().DataPointsFilterRegistrar.create("SamplingSurfaceNormalDataPointsFilter", filterParams);
					}
					else if (filterName == "spdf")
					{
						paramName = "radius";
						filterParams["k"] = "50";
						filterParams["sigma"] = "0.2";
						filterParams["itMax"] = "15";
						filterParams["keepNormals"] = "0";
						filterParams["keepLabels"] = "0";
						filterParams["keepLambdas"] = "0";
						filterParams["keepTensors"] = "0";
						filterParams[paramName] = std::to_string(param_value);
						filter = PM::get().DataPointsFilterRegistrar.create("SpectralDecompositionDataPointsFilter", filterParams);
					}
					else if (filterName == "voxelGrid")
					{
						filterParams["vSizeX"] = std::to_string(param_value);
						filterParams["vSizeY"] = std::to_string(param_value);
						filterParams["vSizeZ"] = std::to_string(param_value);
						filter = PM::get().DataPointsFilterRegistrar.create("VoxelGridDataPointsFilter", filterParams);
					}

					auto localPointCloudInSensorFrame = map.getMapInSensorFrame(currentPose);
					auto filteredMap = getOptimallyFilteredCloud(filterParams, filter, paramName, localPointCloudInSensorFrame);
					map.setLocalMap(filteredMap, currentPose, true);
				}
			}
			mapUpdateDuration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
			float achievedCR = (1.0 - ((float)map.getLocalPointCloud().getNbPoints() / (float)comulativeNbScanPoints));
//			std::cout << "Desired CR: " << desiredCompressionRatio << " Used: " << compressionRatio << " Achieved: " << achievedCR << std::endl;

		}
		else
		{

			start = std::chrono::high_resolution_clock::now();
			map.applyPostFilters(currentPose, mapPostFilters);
			stop = std::chrono::high_resolution_clock::now();
			mapUpdateDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
		}
	}
	return mapUpdateDuration;
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
