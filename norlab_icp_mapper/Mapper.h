#ifndef MAPPER_H
#define MAPPER_H

#include <pointmatcher/PointMatcher.h>
#include "Map.h"
#include "Trajectory.h"
#include <future>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include "MapperModules/MapperModule.h"

namespace norlab_icp_mapper
{
	class Mapper
	{
	private:
		typedef PointMatcher<float> PM;

        const std::string defaultMapUpdateCondition = "distance";
        const float defaultMapUpdateDistance = 1.0;

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

        void updateMap(const PM::DataPoints& currentInput, const PM::TransformationParameters& currentPose,
					   const std::chrono::time_point<std::chrono::steady_clock>& currentTimeStamp);
        void validateYamlKeys(const YAML::Node& node, const std::vector<std::string>& validKeys);
        void setDefaultMapUpdateConfig();
        void setDefaultMapperConfig();
        bool shouldUpdateMap(const std::chrono::time_point<std::chrono::steady_clock>& currentTime, const PM::TransformationParameters& currentPose,
                             const float& currentOverlap) const;

	public:
		Mapper(const std::string& configFilePath, const bool& is3D, const bool& isOnline,
			   const bool& isMapping, const bool& saveMapCellsOnHardDrive);
		void processInput(const PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& estimatedPose,
						  const std::chrono::time_point<std::chrono::steady_clock>& timeStamp);
		PM::DataPoints getMap();
		void setMap(const PM::DataPoints& newMap);
		bool getNewLocalMap(PM::DataPoints& mapOut);
		PM::TransformationParameters getPose();
		bool getIsMapping() const;
		void setIsMapping(const bool& newIsMapping);
		Trajectory getTrajectory();
        void setDefaultMapperModule();
		void loadYamlConfig(const std::string& configFilePath);

    private:
        DEF_REGISTRAR(MapperModule)
        void fillRegistrar();
    };
}

#endif
