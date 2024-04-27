#include "Mapper.h"
#include <fstream>
#include <chrono>
#include "FactorGraph.h"
#include "utils.hpp"

norlab_icp_mapper::Mapper::Mapper(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath,
                                  const std::string& mapPostFiltersConfigFilePath, const std::string& mapUpdateCondition, const float& mapUpdateOverlap,
                                  const float& mapUpdateDelay, const float& mapUpdateDistance, const float& minDistNewPoint, const float& sensorMaxRange,
                                  const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle, const float& epsilonA,
                                  const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& computeProbDynamic, const bool& isMapping,
                                  const bool& saveMapCellsOnHardDrive, const PM::TransformationParameters& imuToLidar):
        mapUpdateCondition(mapUpdateCondition),
        mapUpdateOverlap(mapUpdateOverlap),
        mapUpdateDelay(mapUpdateDelay),
        mapUpdateDistance(mapUpdateDistance),
        is3D(is3D),
        isMapping(isMapping),
        imuToLidar(imuToLidar),
        map(minDistNewPoint, sensorMaxRange, priorDynamic, thresholdDynamic, beamHalfAngle, epsilonA, epsilonD, alpha, beta, is3D,
            computeProbDynamic, saveMapCellsOnHardDrive, icp, icpMapLock),
        trajectory(is3D ? 3 : 2),
        transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
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

void norlab_icp_mapper::Mapper::processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& poseAtStartOfScan,
                                             const Eigen::Matrix<float, 3, 1>& velocityAtStartOfScan, const std::vector<ImuMeasurement>& imuMeasurements,
                                             const std::chrono::time_point<std::chrono::steady_clock>& timeStampAtStartOfScan,
                                             const std::chrono::time_point<std::chrono::steady_clock>& timeStampAtEndOfScan)
{
    PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);
    inputFilters.apply(filteredInputInSensorFrame);

    FactorGraph factorGraph(poseAtStartOfScan, velocityAtStartOfScan, timeStampAtStartOfScan, timeStampAtEndOfScan, imuMeasurements, imuToLidar);
    std::vector<StampedState> optimizedStates = factorGraph.getPredictedStates();

    if(map.isLocalPointCloudEmpty())
    {
        map.updatePose(poseAtStartOfScan);

        updateMap(deskew(filteredInputInSensorFrame, timeStampAtStartOfScan, optimizedStates), poseAtStartOfScan, timeStampAtStartOfScan);
    }
    else
    {
        icpMapLock.lock();
        PM::DataPoints reference = map.getIcpMap();
        icp.readingStepDataPointsFilters.init();
        PM::TransformationParameters T_iter = PM::Matrix::Identity(4, 4);
        bool iterate(true);
        icp.transformationCheckers.init(T_iter, iterate);
        size_t iterationCount(0);
        while(iterate)
        {
            PM::DataPoints stepReading(deskew(filteredInputInSensorFrame, timeStampAtStartOfScan, optimizedStates));
            icp.readingDataPointsFilters.init();
            icp.readingDataPointsFilters.apply(stepReading);
            icp.readingStepDataPointsFilters.apply(stepReading);
            const PM::Matches matches(icp.matcher->findClosests(stepReading));
            const PM::OutlierWeights outlierWeights(icp.outlierFilters.compute(stepReading, reference, matches));
            icp.inspector->dumpIteration(iterationCount, T_iter, reference, stepReading, matches, outlierWeights, icp.transformationCheckers);
            T_iter = icp.errorMinimizer->compute(stepReading, reference, outlierWeights, matches) * T_iter;
            optimizedStates = factorGraph.optimize(T_iter);
            try
            {
                icp.transformationCheckers.check(T_iter, iterate);
            }
            catch(...)
            {
                iterate = false;
            }
            ++iterationCount;
        }
        icpMapLock.unlock();

        map.updatePose(poseAtStartOfScan);

        if(shouldUpdateMap(timeStampAtStartOfScan, poseAtStartOfScan, icp.errorMinimizer->getOverlap()))
        {
            updateMap(deskew(filteredInputInSensorFrame, timeStampAtStartOfScan, optimizedStates), poseAtStartOfScan, timeStampAtStartOfScan);
        }
    }
    StampedState stateAtEndOfScan = optimizedStates[optimizedStates.size() - 1];

    poseLock.lock();
    pose = stateAtEndOfScan.pose;
    poseLock.unlock();

    velocityLock.lock();
    velocity = stateAtEndOfScan.velocity;
    velocityLock.unlock();

    trajectoryLock.lock();
    if(trajectory.getSize() == 0)
    {
        trajectory.addPose(poseAtStartOfScan, timeStampAtStartOfScan);
    }
    trajectory.addPose(stateAtEndOfScan.pose, stateAtEndOfScan.timeStamp);
    trajectoryLock.unlock();
}

bool norlab_icp_mapper::Mapper::shouldUpdateMap(const std::chrono::time_point<std::chrono::steady_clock>& currentTime,
                                                const PM::TransformationParameters& currentPose, const float& currentOverlap) const
{
    if(!isMapping.load())
    {
        return false;
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

void norlab_icp_mapper::Mapper::updateMap(const PM::DataPoints& input, const PM::TransformationParameters& pose,
                                          const std::chrono::time_point<std::chrono::steady_clock>& timeStamp)
{
    lastTimeMapWasUpdated = timeStamp;
    lastPoseWhereMapWasUpdated = pose;

    map.updateLocalPointCloud(input, pose, mapPostFilters);
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

Eigen::Matrix<float, 3, 1> norlab_icp_mapper::Mapper::getVelocity()
{
    std::lock_guard<std::mutex> lock(velocityLock);
    return velocity;
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
