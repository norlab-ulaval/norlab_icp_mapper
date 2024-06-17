#include "Mapper.h"
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

#include <fstream>

const std::string TRAJECTORY_FILE_PATH = "/home/sp/Desktop/deskewing_icp_traj.csv";

void saveFinalStates(const std::vector<StampedState>& states, const bool& resetFile = false)
{
    std::ofstream trajectoryFile;
    if(resetFile)
    {
        trajectoryFile = std::ofstream(TRAJECTORY_FILE_PATH);
        trajectoryFile << "timestamp,t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33" << std::endl;
    }
    else
    {
        trajectoryFile = std::ofstream(TRAJECTORY_FILE_PATH, std::ios::app);
    }
    for(const StampedState& state: states)
    {
        trajectoryFile << state.timeStamp.time_since_epoch().count() << "," << state.pose(0, 0) << "," << state.pose(0, 1) << "," << state.pose(0, 2) << "," << state.pose(0, 3)
                       << "," << state.pose(1, 0) << "," << state.pose(1, 1) << "," << state.pose(1, 2) << "," << state.pose(1, 3)
                       << "," << state.pose(2, 0) << "," << state.pose(2, 1) << "," << state.pose(2, 2) << "," << state.pose(2, 3)
                       << "," << state.pose(3, 0) << "," << state.pose(3, 1) << "," << state.pose(3, 2) << "," << state.pose(3, 3) << std::endl;
    }
    trajectoryFile.close();
}

int scanCounter = 0;
int TARGET_SCAN = -1;

void norlab_icp_mapper::Mapper::processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& poseAtStartOfScan,
                                             const Eigen::Matrix<float, 3, 1>& velocityAtStartOfScan, const std::vector<ImuMeasurement>& imuMeasurements,
                                             const std::chrono::time_point<std::chrono::steady_clock>& timeStampAtStartOfScan,
                                             const std::chrono::time_point<std::chrono::steady_clock>& timeStampAtEndOfScan)
{
    ++scanCounter;

    PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);
    inputFilters.apply(filteredInputInSensorFrame);

    std::vector<StampedState> optimizedStates;
    if(map.isLocalPointCloudEmpty())
    {
        FactorGraph factorGraph(poseAtStartOfScan, velocityAtStartOfScan, timeStampAtStartOfScan, timeStampAtEndOfScan, imuMeasurements, imuToLidar);
        optimizedStates = factorGraph.getPredictedStates();
        saveFinalStates(optimizedStates, true);

        if(scanCounter == TARGET_SCAN)
        {
            exit(0);
        }

        map.updatePose(poseAtStartOfScan);

        updateMap(deskew(filteredInputInSensorFrame, timeStampAtStartOfScan, optimizedStates), poseAtStartOfScan, timeStampAtStartOfScan);
    }
    else
    {
        icpMapLock.lock();
        PM::DataPoints reference(map.getIcpMap());
        const PM::Vector meanRef = reference.features.rowwise().sum() / reference.features.cols();
        PM::TransformationParameters T_refIn_refMean = PM::Matrix::Identity(4, 4);
        T_refIn_refMean.block(0, 3, 3, 1) = meanRef.head(3);
        reference.features.topRows(3).colwise() -= meanRef.head(3);
        std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create(icp.matcher->className, icp.matcher->parameters);
        matcher->init(reference);

        PM::TransformationParameters poseAtStartOfScan_refMean = T_refIn_refMean.inverse() * poseAtStartOfScan;
        FactorGraph factorGraph(poseAtStartOfScan_refMean, velocityAtStartOfScan, timeStampAtStartOfScan, timeStampAtEndOfScan, imuMeasurements, imuToLidar);
        std::vector<StampedState> optimizedStates_refMean = factorGraph.getPredictedStates();

        PM::DataPoints reading(filteredInputInSensorFrame);
        icp.readingDataPointsFilters.init();
        icp.readingDataPointsFilters.apply(reading);

        icp.readingStepDataPointsFilters.init();
        PM::TransformationParameters T_iter = PM::Matrix::Identity(4, 4);
        bool iterate(true);
        icp.transformationCheckers.init(T_iter, iterate);
        size_t iterationCount(0);
        if(scanCounter == TARGET_SCAN)
        {
            std::cout << "==== ICP residual ====" << std::endl;
            PM::DataPoints stepReading(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean));
            const PM::Matches matches(matcher->findClosests(stepReading));
            const PM::OutlierWeights outlierWeights(icp.outlierFilters.compute(stepReading, reference, matches));
            std::cout << icp.errorMinimizer->getResidualError(stepReading, reference, outlierWeights, matches) << std::endl;
        }
        while(iterate)
        {
            PM::DataPoints stepReading(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean));
            icp.readingStepDataPointsFilters.apply(stepReading);
            const PM::Matches matches(matcher->findClosests(stepReading));
            const PM::OutlierWeights outlierWeights(icp.outlierFilters.compute(stepReading, reference, matches));
            if(scanCounter == TARGET_SCAN)
            {
                icp.inspector->dumpIteration(iterationCount, T_iter, reference, stepReading, matches, outlierWeights, icp.transformationCheckers);
            }
            T_iter = icp.errorMinimizer->compute(stepReading, reference, outlierWeights, matches) * T_iter;
            optimizedStates_refMean = factorGraph.optimize(T_iter, iterationCount, scanCounter == TARGET_SCAN);
            if(scanCounter == TARGET_SCAN)
            {
                std::cout << icp.errorMinimizer->getResidualError(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean), reference, outlierWeights, matches)
                          << std::endl;
            }
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
        if(scanCounter == TARGET_SCAN)
        {
            std::cout << "======================" << std::endl;
        }
        icpMapLock.unlock();

        optimizedStates = applyTransformationToStates(T_refIn_refMean, optimizedStates_refMean);
        saveFinalStates(optimizedStates);

        if(scanCounter == TARGET_SCAN)
        {
            exit(0);
        }

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
