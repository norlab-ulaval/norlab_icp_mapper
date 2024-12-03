#include "Mapper.h"
#include <chrono>
#include "FactorGraph.h"
#include "utils.hpp"

norlab_icp_mapper::Mapper::Mapper(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath,
                                  const std::string& mapPostFiltersConfigFilePath, const std::string& mapUpdateCondition, const float& mapUpdateOverlap,
                                  const float& mapUpdateDelay, const float& mapUpdateDistance, const float& minDistNewPoint, const float& sensorMaxRange,
                                  const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle, const float& epsilonA,
                                  const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& computeProbDynamic, const bool& isMapping,
                                  const bool& saveMapCellsOnHardDrive, const PM::TransformationParameters& imuToLidar, const bool& reconstructContinuousTrajectory,
                                  const float& linearVelocityNoise):
        mapUpdateCondition(mapUpdateCondition),
        mapUpdateOverlap(mapUpdateOverlap),
        mapUpdateDelay(mapUpdateDelay),
        mapUpdateDistance(mapUpdateDistance),
        is3D(is3D),
        isMapping(isMapping),
        imuToLidar(imuToLidar),
        map(minDistNewPoint, sensorMaxRange, priorDynamic, thresholdDynamic, beamHalfAngle, epsilonA, epsilonD, alpha, beta, is3D,
            computeProbDynamic, saveMapCellsOnHardDrive),
        transformation(PM::get().TransformationRegistrar.create("RigidTransformation")),
        reconstructContinuousTrajectory(reconstructContinuousTrajectory),
        linearVelocityNoise(linearVelocityNoise)
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
        std::ifstream ifsDynamic(icpConfigFilePath.c_str());
        icpDynamic.loadFromYaml(ifsDynamic);
        ifsDynamic.close();
        std::ifstream ifsStatic(icpConfigFilePath.c_str());
        icpStatic.loadFromYaml(ifsStatic);
        ifsStatic.close();
    }
    else
    {
        icpDynamic.setDefault();
        icpStatic.setDefault();
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

int scanCounter = 0;
int TARGET_SCAN = -1;
const std::string DEBUG_FOLDER = "/home/sp/data/iros2024/debug/continuous_traj_run_1_decomposed/";

void saveInitialPose(const PM::TransformationParameters& initialPose, const std::string& fileName)
{
    std::ofstream ofstream(fileName);
    ofstream << "t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33" << std::endl;
    ofstream << initialPose(0, 0) << "," << initialPose(0, 1) << "," << initialPose(0, 2) << "," << initialPose(0, 3) << "," <<
             initialPose(1, 0) << "," << initialPose(1, 1) << "," << initialPose(1, 2) << "," << initialPose(1, 3) << "," <<
             initialPose(2, 0) << "," << initialPose(2, 1) << "," << initialPose(2, 2) << "," << initialPose(2, 3) << "," <<
             initialPose(3, 0) << "," << initialPose(3, 1) << "," << initialPose(3, 2) << "," << initialPose(3, 3) << std::endl;
    ofstream.close();
}

void saveInitialVelocity(const Eigen::Matrix<float, 3, 1>& initialVelocity, const std::string& fileName)
{
    std::ofstream ofstream(fileName);
    ofstream << "v0,v1,v2" << std::endl;
    ofstream << initialVelocity(0, 0) << "," << initialVelocity(1, 0) << "," << initialVelocity(2, 0) << std::endl;
    ofstream.close();
}

void saveImuMeasurement(const ImuMeasurement& measurement, const std::string& fileName)
{
    std::ofstream ofstream(fileName);
    ofstream << "linear_acceleration.x,linear_acceleration.y,linear_acceleration.z,angular_velocity.x,angular_velocity.y,angular_velocity.z" << std::endl;
    ofstream << measurement.linearAcceleration(0) << "," << measurement.linearAcceleration(1) << "," << measurement.linearAcceleration(2) << "," << measurement.angularVelocity(0)
             << "," << measurement.angularVelocity(1) << "," << measurement.angularVelocity(2) << std::endl;
    ofstream.close();
}

void saveFinalTimestamp(const std::chrono::time_point<std::chrono::steady_clock>& finalTimestamp, const std::string& fileName)
{
    std::ofstream ofstream(fileName);
    ofstream << finalTimestamp.time_since_epoch().count() << std::endl;
    ofstream.close();
}

void saveLidarState(const StampedState& lidarState, const std::string& fileName)
{
    std::ofstream ofstream(fileName);
    ofstream << "t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33,v0,v1,v2" << std::endl;
    ofstream << lidarState.pose(0, 0) << "," << lidarState.pose(0, 1) << "," << lidarState.pose(0, 2) << "," << lidarState.pose(0, 3) << "," <<
             lidarState.pose(1, 0) << "," << lidarState.pose(1, 1) << "," << lidarState.pose(1, 2) << "," << lidarState.pose(1, 3) << "," <<
             lidarState.pose(2, 0) << "," << lidarState.pose(2, 1) << "," << lidarState.pose(2, 2) << "," << lidarState.pose(2, 3) << "," <<
             lidarState.pose(3, 0) << "," << lidarState.pose(3, 1) << "," << lidarState.pose(3, 2) << "," << lidarState.pose(3, 3) << "," <<
             lidarState.velocity(0, 0) << "," << lidarState.velocity(1, 0) << "," << lidarState.velocity(2, 0) << std::endl;
    ofstream.close();
}

void norlab_icp_mapper::Mapper::processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& poseAtStartOfScan,
                                             const Eigen::Matrix<float, 3, 1>& velocityAtStartOfScan, const std::vector<ImuMeasurement>& imuMeasurements,
                                             const std::chrono::time_point<std::chrono::steady_clock>& timeStampAtStartOfScan,
                                             const std::chrono::time_point<std::chrono::steady_clock>& timeStampAtEndOfScan)
{
    ++scanCounter;

    PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);
    inputFilters.apply(filteredInputInSensorFrame);

//    saveInitialPose(poseAtStartOfScan, DEBUG_FOLDER + "initial_pose_" + std::to_string(timeStampAtStartOfScan.time_since_epoch().count()) + ".csv");
//    saveInitialVelocity(velocityAtStartOfScan, DEBUG_FOLDER + "initial_velocity_" + std::to_string(timeStampAtStartOfScan.time_since_epoch().count()) + ".csv");
//    saveFinalTimestamp(timeStampAtEndOfScan, DEBUG_FOLDER + "final_timestamp_" + std::to_string(timeStampAtStartOfScan.time_since_epoch().count()) + ".csv");
//    for(int i = 0; i < imuMeasurements.size(); ++i)
//    {
//        saveImuMeasurement(imuMeasurements[i], DEBUG_FOLDER + "imu_measurement_" + std::to_string(imuMeasurements[i].timeStamp.time_since_epoch().count()) + ".csv");
//    }
//    filteredInputInSensorFrame.save(DEBUG_FOLDER + "scan_" + std::to_string(timeStampAtStartOfScan.time_since_epoch().count()) + ".csv");
//    map.getLocalPointCloud().save(DEBUG_FOLDER + "map_" + std::to_string(timeStampAtStartOfScan.time_since_epoch().count()) + ".csv");
//    for(int i = 0; i < intraScanTrajectory.size(); ++i)
//    {
//        saveLidarState(intraScanTrajectory[i], DEBUG_FOLDER + "lidar_state_" + std::to_string(intraScanTrajectory[i].timeStamp.time_since_epoch().count()) + ".csv");
//    }

    std::vector<StampedState> optimizedStates;
    if(map.isLocalPointCloudEmpty())
    {
        FactorGraph factorGraph(poseAtStartOfScan, velocityAtStartOfScan, timeStampAtStartOfScan, timeStampAtEndOfScan, imuMeasurements, imuToLidar, true, linearVelocityNoise);
        optimizedStates = factorGraph.getPredictedStates();

        if(scanCounter == TARGET_SCAN)
        {
            exit(0);
        }

        map.updatePose(poseAtStartOfScan);

        updateMap(deskew(filteredInputInSensorFrame, timeStampAtStartOfScan, optimizedStates), poseAtStartOfScan, timeStampAtStartOfScan);
    }
    else
    {
        PM::DataPoints reference(map.getLocalPointCloud());
        icpDynamic.referenceDataPointsFilters.init();
        icpDynamic.referenceDataPointsFilters.apply(reference);
        const PM::Vector meanRef = reference.features.rowwise().sum() / reference.features.cols();
        PM::TransformationParameters T_refIn_refMean = PM::Matrix::Identity(4, 4);
        T_refIn_refMean.block(0, 3, 3, 1) = meanRef.head(3);
        reference.features.topRows(3).colwise() -= meanRef.head(3);
        std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create(icpDynamic.matcher->className, icpDynamic.matcher->parameters);
        matcher->init(reference);

        PM::TransformationParameters poseAtStartOfScan_refMean = T_refIn_refMean.inverse() * poseAtStartOfScan;
        FactorGraph factorGraph(poseAtStartOfScan_refMean, velocityAtStartOfScan, timeStampAtStartOfScan, timeStampAtEndOfScan, imuMeasurements, imuToLidar, true,
                                linearVelocityNoise);
        std::vector<StampedState> optimizedStates_refMean = factorGraph.getPredictedStates();

        PM::DataPoints reading(filteredInputInSensorFrame);
        icpDynamic.readingDataPointsFilters.init();
        icpDynamic.readingDataPointsFilters.apply(reading);

        icpDynamic.readingStepDataPointsFilters.init();
        PM::TransformationParameters T_iter_dynamic = PM::Matrix::Identity(4, 4);
        bool iterateDynamic(true);
        icpDynamic.transformationCheckers.init(T_iter_dynamic, iterateDynamic);
        size_t iterationCountDynamic(0);
        if(scanCounter == TARGET_SCAN)
        {
            std::cout << "==== Dynamic ICP residual ====" << std::endl;
            PM::DataPoints stepReadingDynamic(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean));
            PM::Matches matchesDynamic(matcher->findClosests(stepReadingDynamic));
            PM::OutlierWeights outlierWeightsDynamic(icpDynamic.outlierFilters.compute(stepReadingDynamic, reference, matchesDynamic));
            std::cout << icpDynamic.errorMinimizer->getResidualError(stepReadingDynamic, reference, outlierWeightsDynamic, matchesDynamic) << std::endl;
        }
        while(iterateDynamic)
        {
            PM::DataPoints stepReadingDynamic(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean));
            icpDynamic.readingStepDataPointsFilters.apply(stepReadingDynamic);
            PM::Matches matchesDynamic(matcher->findClosests(stepReadingDynamic));
            PM::OutlierWeights outlierWeightsDynamic(icpDynamic.outlierFilters.compute(stepReadingDynamic, reference, matchesDynamic));
            if(scanCounter == TARGET_SCAN)
            {
                icpDynamic.inspector->dumpIteration(iterationCountDynamic, T_iter_dynamic, reference, stepReadingDynamic, matchesDynamic, outlierWeightsDynamic,
                                                    icpDynamic.transformationCheckers);
            }

            PM::TransformationParameters T_iter_static = PM::Matrix::Identity(4, 4);
            bool iterateStatic(true);
            icpStatic.transformationCheckers.init(T_iter_static, iterateStatic);
            size_t iterationCountStatic(0);
            if(scanCounter == TARGET_SCAN)
            {
                std::cout << "==== Static ICP residual ====" << std::endl;
                PM::DataPoints stepReadingStatic(stepReadingDynamic);
                PM::Matches matchesStatic(matcher->findClosests(stepReadingStatic));
                PM::OutlierWeights outlierWeightsStatic(icpStatic.outlierFilters.compute(stepReadingStatic, reference, matchesStatic));
                std::cout << icpStatic.errorMinimizer->getResidualError(stepReadingStatic, reference, outlierWeightsStatic, matchesStatic) << std::endl;
            }
            while(iterateStatic)
            {
                PM::DataPoints stepReadingStatic(stepReadingDynamic);
                icpStatic.transformations.apply(stepReadingStatic, T_iter_static);
                PM::Matches matchesStatic(matcher->findClosests(stepReadingStatic));
                PM::OutlierWeights outlierWeightsStatic(icpStatic.outlierFilters.compute(stepReadingStatic, reference, matchesStatic));
                PM::TransformationParameters correction = icpStatic.errorMinimizer->compute(stepReadingStatic, reference, outlierWeightsStatic, matchesStatic);
                T_iter_static = correction * T_iter_static;
                if(scanCounter == TARGET_SCAN)
                {
                    icpStatic.transformations.apply(stepReadingStatic, correction);
                    std::cout << icpStatic.errorMinimizer->getResidualError(stepReadingStatic, reference, outlierWeightsStatic, matchesStatic) << std::endl;
                }
                try
                {
                    icpStatic.transformationCheckers.check(T_iter_static, iterateStatic);
                }
                catch(...)
                {
                    iterateStatic = false;
                }
                ++iterationCountStatic;
            }
            if(scanCounter == TARGET_SCAN)
            {
                std::cout << "Static ICP converged in " << iterationCountStatic << " iterations" << std::endl;
                std::cout << "=============================" << std::endl;
            }

            T_iter_dynamic = T_iter_static * T_iter_dynamic;
            optimizedStates_refMean = factorGraph.optimize(T_iter_dynamic, iterationCountDynamic, true);
            if(scanCounter == TARGET_SCAN)
            {
                std::cout << icpDynamic.errorMinimizer->getResidualError(deskew(reading, timeStampAtStartOfScan, optimizedStates_refMean), reference, outlierWeightsDynamic,
                                                                         matchesDynamic) << std::endl;
            }
            try
            {
                icpDynamic.transformationCheckers.check(T_iter_dynamic, iterateDynamic);
            }
            catch(...)
            {
                iterateDynamic = false;
            }
            ++iterationCountDynamic;
        }
        if(scanCounter == TARGET_SCAN)
        {
            std::cout << "Dynamic ICP converged in " << iterationCountDynamic << " iterations" << std::endl;
            std::cout << "=============================" << std::endl;
        }

        optimizedStates = applyTransformationToStates(T_refIn_refMean, optimizedStates_refMean);

        if(scanCounter == TARGET_SCAN)
        {
            exit(0);
        }

        map.updatePose(poseAtStartOfScan);

        if(shouldUpdateMap(timeStampAtStartOfScan, poseAtStartOfScan, icpStatic.errorMinimizer->getOverlap()))
        {
            updateMap(deskew(filteredInputInSensorFrame, timeStampAtStartOfScan, optimizedStates), poseAtStartOfScan, timeStampAtStartOfScan);
        }
    }

    trajectoryLock.lock();
    intraScanTrajectory = optimizedStates;
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
}

bool norlab_icp_mapper::Mapper::getNewLocalMap(PM::DataPoints& mapOut)
{
    return map.getNewLocalPointCloud(mapOut);
}

norlab_icp_mapper::Mapper::PM::TransformationParameters norlab_icp_mapper::Mapper::getPose()
{
    std::lock_guard<std::mutex> lock(trajectoryLock);
    return intraScanTrajectory[intraScanTrajectory.size() - 1].pose;
}

Eigen::Matrix<float, 3, 1> norlab_icp_mapper::Mapper::getVelocity()
{
    std::lock_guard<std::mutex> lock(trajectoryLock);
    return intraScanTrajectory[intraScanTrajectory.size() - 1].velocity;
}

std::vector<StampedState> norlab_icp_mapper::Mapper::getIntraScanTrajectory()
{
    std::lock_guard<std::mutex> lock(trajectoryLock);
    return intraScanTrajectory;
}

bool norlab_icp_mapper::Mapper::getIsMapping() const
{
    return isMapping.load();
}

void norlab_icp_mapper::Mapper::setIsMapping(const bool& newIsMapping)
{
    isMapping.store(newIsMapping);
}
