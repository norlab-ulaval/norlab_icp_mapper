#include "Mapper.h"
#include "MapperModules/PointDistanceMapperModule.h"
#include "MapperModules/OctreeMapperModule.h"
#include "MapperModules/ComputeDynamicsMapperModule.h"
#include <fstream>
#include <chrono>
#include <yaml-cpp/node/iterator.h>

void norlab_icp_mapper::Mapper::fillRegistrar() {
    ADD_TO_REGISTRAR(MapperModule, PointDistanceMapperModule, PointDistanceMapperModule);
    ADD_TO_REGISTRAR(MapperModule, OctreeMapperModule, OctreeMapperModule);
    ADD_TO_REGISTRAR(MapperModule, ComputeDynamicsMapperModule, ComputeDynamicsMapperModule);
}

norlab_icp_mapper::Mapper::Mapper(const std::string& configFilePath,
                                  const bool& is3D, const bool& isOnline, const bool& isMapping, const bool& saveMapCellsOnHardDrive):
		is3D(is3D),
		isOnline(isOnline),
		isMapping(isMapping),
		map(is3D, isOnline, saveMapCellsOnHardDrive, icp, icpMapLock),
		trajectory(is3D ? 3 : 2),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
{
    fillRegistrar();
	loadYamlConfig(configFilePath);

	PM::Parameters radiusFilterParams;
	radiusFilterParams["dim"] = "-1";
	radiusFilterParams["dist"] = std::to_string(map.getSensorMaxRange());
	radiusFilterParams["removeInside"] = "0";
	radiusFilter = PM::get().DataPointsFilterRegistrar.create("DistanceLimitDataPointsFilter", radiusFilterParams);

}

void norlab_icp_mapper::Mapper::validateYamlKeys(const YAML::Node& node, const std::vector<std::string>& validKeys)
{
    std::unordered_set<std::string> encounteredKeys;
    if (!node.IsMap())
    {
        throw YAML::Exception(node.Mark(), "Expected a YAML Map node.");
    }

    // Iterate over the map and check if each key is unique
    for (const auto& kv : node) {
        const auto key = kv.first.as<std::string>();

        if (encounteredKeys.count(key) > 0)
        {
            throw YAML::Exception(kv.second.Mark(), "Duplicated key: " + key);
        }
        if (std::find(validKeys.begin(), validKeys.end(), key) == validKeys.end())
        {
            throw YAML::Exception(kv.second.Mark(), "Invalid key: " + key);
        }
        encounteredKeys.insert(key);
    }
}

void norlab_icp_mapper::Mapper::loadYamlConfig(const std::string& configFilePath)
{
    std::ifstream ifs(configFilePath.c_str());
    if (ifs.fail())
    {
        throw std::runtime_error("The input config file " + configFilePath + " does not exist");
    }

    YAML::Node node = YAML::Load(ifs);

    validateYamlKeys(node, std::vector<std::string>{"icp", "input", "post", "mapper"});
    if (node["icp"])
    {
        icp.loadFromYamlNode(node["icp"]);
    }
    else
    {
        std::cout << "icp config not found, using default" << std::endl;
        icp.setDefault();
    }

    if(node["input"])
    {
        inputFilters = PM::DataPointsFilters(node["input"]);
    }
    else
    {
        std::cout << "Input config not found, using empty configuration." << std::endl;
        // FIXME add default input filters that "just work"
    }

    if(node["post"])
    {
        mapPostFilters = PM::DataPointsFilters(node["post"]);
    }
    else
    {
        std::cout << "Post config not found, using empty configuration." << std::endl;
        // FIXME add default post filters that "just work"
    }

    if (node["mapper"])
    {
        YAML::Node mapperNode = node["mapper"];
        
        if(mapperNode["updateCondition"])
        {
            YAML::Node updateConditionNode = mapperNode["updateCondition"];
            validateYamlKeys(updateConditionNode, std::vector<std::string>{"type", "value"});
            if(! updateConditionNode["type"])
            {
                throw YAML::Exception(YAML::Mark::null_mark(), "Missing key: type"); // TODO add yaml mark
            }
            if(! updateConditionNode["value"])
            {
                throw YAML::Exception(YAML::Mark::null_mark(), "Missing key: value"); // TODO add yaml mark
            }

            mapUpdateCondition = updateConditionNode["type"].as<std::string>();
            if(mapUpdateCondition == "distance")
            {
                mapUpdateDistance = updateConditionNode["value"].as<float>();
            }
            if(mapUpdateCondition == "overlap")
            {
                mapUpdateOverlap = updateConditionNode["value"].as<float>();
            }
            if(mapUpdateCondition == "delay")
            {
                mapUpdateDelay = updateConditionNode["value"].as<float>();
            }
        }
        else
        {
            std::cout << "Mapper update condition not found, using default configuration." << std::endl;
            setDefaultMapUpdateConfig();
        }

        if(mapperNode["sensorMaxRange"])
        {
            map.setSensorMaxRange(mapperNode["sensorMaxRange"].as<float>());
        }

        if(mapperNode["mapperModuleVec"])
        {
	        YAML::Node mapperModule = mapperNode["mapperModuleVec"];

            for(YAML::const_iterator moduleIt = mapperModule.begin(); moduleIt != mapperModule.end(); ++moduleIt)
            {
                std::shared_ptr<MapperModule> module = REG(MapperModule).createFromYAML(*moduleIt);
                map.addMapperModule(module);
            }
        }
        else
        {
            std::cout << "mapper module not found, using default" << std::endl;
            setDefaultMapperModule();
        }
    }
    else
    {
        std::cout << "mapper config not found, using default" << std::endl;
        setDefaultMapperConfig();
    }
    ifs.close();
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

		updateMap(input, correctedPose, timeStamp);
	}
	else
	{
		PM::TransformationParameters correction;
		{
			std::lock_guard<std::mutex> icpMapLockGuard(icpMapLock);
			correction = icp(input);
		}
		correctedPose = correction * estimatedPose;

		map.updatePose(correctedPose);

		if(shouldUpdateMap(timeStamp, correctedPose, 0.0))
		{
			updateMap(transformation->compute(input, correction), correctedPose, timeStamp);
		}
	}

	poseLock.lock();
	pose = correctedPose;
	poseLock.unlock();

	int euclideanDim = is3D ? 3 : 2;
	trajectoryLock.lock();
	trajectory.addPose(correctedPose, timeStamp);
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
	lastTimeMapWasUpdated = currentTimeStamp;
	lastPoseWhereMapWasUpdated = currentPose;

	if(isOnline && !map.isLocalPointCloudEmpty())
	{
		mapUpdateFuture = std::async(std::launch::async, &Map::updateLocalPointCloud, &map, currentInput, currentPose, mapPostFilters);
        mapUpdateFuture.get();
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
	trajectory.clear();
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

void norlab_icp_mapper::Mapper::setDefaultMapperModule()
{
    std::map<std::string, PM::Parametrizable::Parameter> params;
    params.insert(std::pair<std::string, PM::Parametrizable::Parameter>("minDistNewPoint", "0.15"));
    PointDistanceMapperModule mapperModule(params);
    this->map.addMapperModule(std::make_shared<PointDistanceMapperModule>(mapperModule));
}

void norlab_icp_mapper::Mapper::setDefaultMapUpdateConfig()
{
    mapUpdateCondition = "distance";
    mapUpdateDistance = 1.0;
}

void norlab_icp_mapper::Mapper::setDefaultMapperConfig()
{
    setDefaultMapUpdateConfig();
    setDefaultMapperModule();
}
