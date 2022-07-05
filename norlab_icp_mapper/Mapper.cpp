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
		getline(sampingParamsFile, filterValue);
		getline(sampingParamsFile, tmp); // compression ratio
		getline(sampingParamsFile, tmp);
		removeWall = std::stoi(tmp) == 1;
		std::cout << "Open file: " << sampingParamsFileName << "\n"
					<< ", Loaded name: " << filterName << ", filterValue: " << filterValue << ", removeWall: " << removeWall << std::endl;
		sampingParamsFile.close(); //close the file object.
		}
	else
	{
		std::cout << "Could not open the file: " << sampingParamsFileName << std::endl;
		throw std::exception();
	}

	std::string seedFileName = "/tmp/seed.txt";
	std::fstream seedFile;
	seedFile.open(seedFileName,std::ios::in); //open a file to perform read operation using file object
	if (seedFile.is_open()){ //checking whether the file is open
		std::string tmp;
		getline(seedFile, tmp);
		seed = std::stoi(tmp);
		std::cout << "Open file " << seedFileName << "Loaded seed: " << seed << std::endl;
		seedFile.close(); //close the file object.
	}
	else
	{
		std::cout << "Could not open the file: " << sampingParamsFileName << std::endl;
		throw std::exception();
	}


//	print to log file
	if (! std::freopen("/tmp/mapper_logger.txt", "w", stderr))
		throw std::exception();
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
	iterationCtr++;

	unsigned int nbPointsScan;
	unsigned int nbPointsScanInputFilters;
	unsigned int nbPointsOriginalMap = map.getLocalPointCloud().getNbPoints();
	int nbPointsMapIcpFilters = -1;
	unsigned int nbPointsMapPostFilters;
	long icpUpdateDuration = -1;
	mapUpdateDuration = -1;

	PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);
	nbPointsScan = filteredInputInSensorFrame.getNbPoints();

	PointMatcherSupport::Parametrizable::Parameters params;
	std::shared_ptr<PM::DataPointsFilter> pm_filter;
	params["xMin"] = "-1.2";
	params["xMax"] = "0.5";
	params["yMin"] = "-0.3";
	params["yMax"] = "0.3";
	params["zMin"] = "-0.3";
	params["zMax"] = "0.6";
	params["removeInside"] = "1";
	pm_filter = PM::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter", params);
	params.clear();
	filteredInputInSensorFrame = pm_filter->filter(filteredInputInSensorFrame);

	params["prob"] = "0.2";
	params["seed"] = std::to_string(seed + iterationCtr);
	pm_filter = PM::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", params);
	params.clear();
	filteredInputInSensorFrame = pm_filter->filter(filteredInputInSensorFrame);

	inputFilters.apply(filteredInputInSensorFrame);
	PM::DataPoints input = transformation->compute(filteredInputInSensorFrame, estimatedPose);

	nbPointsScanInputFilters = input.getNbPoints();
	PM::TransformationParameters correctedPose;
	if(map.isLocalPointCloudEmpty())
	{
		correctedPose = estimatedPose;

		map.updatePose(correctedPose);

		comulativeNbScanPoints += input.getNbPoints();
		updateMap(input, correctedPose, timeStamp);
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

void norlab_icp_mapper::Mapper::updateMap(const PM::DataPoints& currentInput, const PM::TransformationParameters& currentPose,
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
		auto filters = PM::DataPointsFilters();
		auto inputCloud = PM::DataPoints(currentInput);
		if(!filterName.empty() && !filterValue.empty()) {
			PM::Parameters filterParams;
			std::shared_ptr<PM::DataPointsFilter> filter;

			if (filterName == "normalSpace" || filterName == "covariance")
			{
				std::string tmpName;
				if (filterName == "normalSpace")
					tmpName = "NormalSpaceDataPointsFilter";
				else
					tmpName = "CovarianceSamplingDataPointsFilter";

				filterParams["nbSample"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create(tmpName, filterParams);
				filters.push_back(filter);
			}
			else if (filterName == "randomInformed")
			{
// remove wall from current input
				std::string name = "BoundingBoxDataPointsFilter";
				filterParams["xMin"] = "-1000.0";
				filterParams["xMax"] = "-1.0";
				filterParams["yMin"] = "-100.0";
				filterParams["yMax"] = "100.0";
				filterParams["zMin"] = "-1000.0";
				filterParams["zMax"] = "1000.0";
				filterParams["removeInside"] = "1";
				filter = PM::get().DataPointsFilterRegistrar.create(name, filterParams);
				auto start = std::chrono::high_resolution_clock::now();
				inputCloud = filter->filter(currentInput);
				auto stop = std::chrono::high_resolution_clock::now();
				mapUpdateDuration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
				filterParams.clear();

				float compressionRatio = std::stof(filterValue);
				size_t current_map_nb_points = map.getLocalPointCloud().getNbPoints();
				size_t current_scan_nb_points = inputCloud.getNbPoints();

				size_t nb_point_we_want = ((100.0 - compressionRatio) / 100.0) * (float) comulativeNbScanPoints;
				long difference_nb_points = std::abs(long (current_map_nb_points + current_scan_nb_points - nb_point_we_want));
				float prob = 1.0 - static_cast<float>(difference_nb_points) / static_cast<float>(current_scan_nb_points);

				// filter reading point cloud
				filterParams["prob"] = std::to_string(prob);
				filter = PM::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", filterParams);

				start = std::chrono::high_resolution_clock::now();
				inputCloud = filter->filter(inputCloud);
				stop = std::chrono::high_resolution_clock::now();
				mapUpdateDuration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

				filterParams.clear();
				filter = PM::get().DataPointsFilterRegistrar.create("IdentityDataPointsFilter", filterParams);
			}
			else if (filterName == "random")
			{
				filterParams["prob"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", filterParams);
			}
			else if (filterName == "maxDensity")
			{
				filter = getSurfaceNormalFilter();
				filters.push_back(filter);
				filterParams["maxDensity"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("MaxDensityDataPointsFilter", filterParams);
			}
			else if (filterName == "octreeVoxelFirstPoint")
			{
				filterParams["samplingMethod"] = "0";
				filterParams["maxSizeByNode"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
			}
			else if (filterName == "octreeVoxelRandom")
			{
				filterParams["samplingMethod"] = "1";
				filterParams["maxSizeByNode"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
			}
			else if (filterName == "octreeVoxelCentroid")
			{
				filterParams["samplingMethod"] = "2";
				filterParams["maxSizeByNode"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
			}
			else if (filterName == "octreeVoxelMedoid")
			{
				filterParams["samplingMethod"] = "3";
				filterParams["maxSizeByNode"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
			}
			else if (filterName == "octreeFirstPoint")
			{
				filterParams["samplingMethod"] = "0";
				filterParams["maxPointByNode"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
			}
			else if (filterName == "octreeRandom")
			{
				filterParams["samplingMethod"] = "1";
				filterParams["maxPointByNode"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
			}
			else if (filterName == "octreeCentroid")
			{
				filterParams["samplingMethod"] = "2";
				filterParams["maxPointByNode"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
			}
			else if (filterName == "octreeMedoid")
			{
				filterParams["samplingMethod"] = "3";
				filterParams["maxPointByNode"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", filterParams);
			}
			else if (filterName == "samplingSurfaceNormal")
			{
				filterParams["keepNormals"] = "0";
				filterParams["ratio"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("SamplingSurfaceNormalDataPointsFilter", filterParams);
			}
			else if (filterName == "spdf")
			{
				filterParams["k"] = "500";
				filterParams["sigma"] = "0.2";
				filterParams["itMax"] = "15";
				filterParams["keepNormals"] = "0";
				filterParams["keepLabels"] = "0";
				filterParams["keepLambdas"] = "0";
				filterParams["keepTensors"] = "0";
				filterParams["radius"] = filterValue;
				filter = PM::get().DataPointsFilterRegistrar.create("SpectralDecompositionDataPointsFilter", filterParams);
			}

			filters.push_back(filter);
		}
		else
		{
			filters = mapPostFilters;
		}
		// merge scan
		auto start = std::chrono::high_resolution_clock::now();
		map.updateLocalPointCloud(inputCloud, currentPose, mapPostFilters, removeWall);
		auto stop = std::chrono::high_resolution_clock::now();
		mapUpdateDuration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

		// apply map post filters
		start = std::chrono::high_resolution_clock::now();
		map.applyPostFilters(currentPose, filters, false);
		stop = std::chrono::high_resolution_clock::now();
		mapUpdateDuration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
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

std::shared_ptr<PointMatcher<float>::DataPointsFilter> norlab_icp_mapper::Mapper::getSurfaceNormalFilter()
{
	PM::Parameters filterParams;
	filterParams["knn"] = "12";
	filterParams["epsilon"] = "0.0";
	filterParams["keepNormals"] = "1";
	filterParams["keepDensities"] = "1";
	return PM::get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter", filterParams);
}