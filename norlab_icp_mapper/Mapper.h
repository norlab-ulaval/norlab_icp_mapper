#ifndef MAPPER_H
#define MAPPER_H

#include <pointmatcher/PointMatcher.h>
#include "Map.h"
#include "Trajectory.h"
#include <future>
#include <mutex>

namespace norlab_icp_mapper
{
	typedef struct
	{
		unsigned nbPointsInputBeforeFiltering;
		unsigned nbPointsInputAfterFiltering;
		float inputFilteringTime;
		unsigned nbPointsReference;
		float estimatedOverlap;
		float processingTime;
		float processingTimePercentage;
	} DiagnosticInformation;

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
		bool isOnline;
		std::atomic_bool isMapping;
		Map map;
		PM::TransformationParameters pose;
		Trajectory trajectory;
		std::shared_ptr<PM::Transformation> transformation;
		std::shared_ptr<PM::DataPointsFilter> radiusFilter;
		std::chrono::time_point<std::chrono::steady_clock> lastTimeMapWasUpdated;
		PM::TransformationParameters lastPoseWhereMapWasUpdated;
		std::mutex poseLock;
		std::mutex trajectoryLock;
		std::mutex icpMapLock;
		std::future<void> mapUpdateFuture;
		std::chrono::time_point<std::chrono::steady_clock> lastTimeStamp;

		bool shouldUpdateMap(const std::chrono::time_point<std::chrono::steady_clock>& currentTime, const PM::TransformationParameters& currentPose,
							 const float& currentOverlap) const;
		void updateMap(const PM::DataPoints& currentInput, const PM::TransformationParameters& currentPose,
					   const std::chrono::time_point<std::chrono::steady_clock>& currentTimeStamp);

	public:
		Mapper(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath, const std::string& mapPostFiltersConfigFilePath,
			   const std::string& mapUpdateCondition, const float& mapUpdateOverlap, const float& mapUpdateDelay, const float& mapUpdateDistance,
			   const float& minDistNewPoint, const float& sensorMaxRange, const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle,
			   const float& epsilonA, const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& isOnline,
			   const bool& computeProbDynamic, const bool& isMapping, const bool& saveMapCellsOnHardDrive);
		void loadYamlConfig(const std::string& inputFiltersConfigFilePath, const std::string& icpConfigFilePath,
							const std::string& mapPostFiltersConfigFilePath);
		DiagnosticInformation processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& estimatedPose,
						  const std::chrono::time_point<std::chrono::steady_clock>& timeStamp);

		DiagnosticInformation processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& estimatedPose,
                                           const std::chrono::time_point<std::chrono::steady_clock>& timeStamp, bool skip_icp);

		PM::DataPoints getMap();
		void setMap(const PM::DataPoints& newMap);
		void clearMap();
		bool getNewLocalMap(PM::DataPoints& mapOut);
		PM::TransformationParameters getPose();
		bool getIsMapping() const;
		void setIsMapping(const bool& newIsMapping);
		Trajectory getTrajectory();
	};
}

#endif
