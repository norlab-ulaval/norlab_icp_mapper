#ifndef MAPPER_H
#define MAPPER_H

#include <pointmatcher/PointMatcher.h>
#include "Map.h"
#include "ImuMeasurement.h"
#include <mutex>
#include "StampedState.h"

namespace norlab_icp_mapper
{
    class Mapper
    {
    private:
        typedef PointMatcher<float> PM;

        PM::DataPointsFilters inputFilters;
        PM::ICPSequence icp;
        PM::DataPointsFilters mapPostFilters;
        std::string mapUpdateCondition;
        float mapUpdateOverlap;
        float mapUpdateDelay;
        float mapUpdateDistance;
        bool is3D;
        std::atomic_bool isMapping;
        PM::TransformationParameters imuToLidar;
        Map map;
        std::vector<StampedState> intraScanTrajectory;
        Eigen::Matrix<float, 3, 1> velocity;
        std::shared_ptr<PM::Transformation> transformation;
        std::shared_ptr<PM::DataPointsFilter> radiusFilter;
        std::chrono::time_point<std::chrono::steady_clock> lastTimeMapWasUpdated;
        PM::TransformationParameters lastPoseWhereMapWasUpdated;
        std::mutex trajectoryLock;
        std::mutex icpMapLock;

        bool shouldUpdateMap(const std::chrono::time_point<std::chrono::steady_clock>& currentTime, const PM::TransformationParameters& currentPose,
                             const float& currentOverlap) const;
        void updateMap(const PM::DataPoints& input, const PM::TransformationParameters& pose, const std::chrono::time_point<std::chrono::steady_clock>& timeStamp);

    public:
        Mapper(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath, const std::string& mapPostFiltersConfigFilePath,
               const std::string& mapUpdateCondition, const float& mapUpdateOverlap, const float& mapUpdateDelay, const float& mapUpdateDistance,
               const float& minDistNewPoint, const float& sensorMaxRange, const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle,
               const float& epsilonA, const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& computeProbDynamic, const bool& isMapping,
               const bool& saveMapCellsOnHardDrive, const PM::TransformationParameters& imuToLidar);
        void loadYamlConfig(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath,
                            const std::string& mapPostFiltersConfigFilePath);
        void processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& poseAtStartOfScan, const Eigen::Matrix<float, 3, 1>& velocityAtStartOfScan,
                          const std::vector<ImuMeasurement>& imuMeasurements, const std::chrono::time_point<std::chrono::steady_clock>& timeStampAtStartOfScan,
                          const std::chrono::time_point<std::chrono::steady_clock>& timeStampAtEndOfScan);
        PM::DataPoints getMap();
        void setMap(const PM::DataPoints& newMap);
        bool getNewLocalMap(PM::DataPoints& mapOut);
        PM::TransformationParameters getPose();
        Eigen::Matrix<float, 3, 1> getVelocity();
        std::vector<StampedState> getIntraScanTrajectory();
        bool getIsMapping() const;
        void setIsMapping(const bool& newIsMapping);
    };
}

#endif
