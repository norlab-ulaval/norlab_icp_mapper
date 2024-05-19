#ifndef MAP_H
#define MAP_H

#include <pointmatcher/PointMatcher.h>
#include <thread>
#include <mutex>
#include <list>
#include <unordered_set>
#include "CellManager.h"
#include "MapperModules/MapperModule.h"

namespace norlab_icp_mapper
{
	class Map
	{
	private:
		typedef PointMatcher<float> PM;

		typedef struct Update
		{
			int startRow;
			int endRow;
			int startColumn;
			int endColumn;
			int startAisle;
			int endAisle;
			bool load;
		} Update;

		const int BUFFER_SIZE = 2;
		const float CELL_SIZE = 20.0;
        const int INITIAL_CELL_NB_POINTS_WHEN_UNLOADING = 100;

		float sensorMaxRange = 200;
		bool is3D;
		bool isOnline;
		PM::ICPSequence& icp;
		std::mutex& icpMapLock;
		PM::DataPoints localPointCloud;
		std::mutex localPointCloudLock;
		std::unique_ptr<CellManager> cellManager;
		std::mutex cellManagerLock;
		std::unordered_set<std::string> loadedCellIds;
		std::shared_ptr<PM::Transformation> transformation;
		int inferiorRowLastUpdateIndex{};
		int superiorRowLastUpdateIndex{};
		int inferiorColumnLastUpdateIndex{};
		int superiorColumnLastUpdateIndex{};
		int inferiorAisleLastUpdateIndex{};
		int superiorAisleLastUpdateIndex{};
		bool newLocalPointCloudAvailable;
		std::atomic_bool localPointCloudEmpty;
		std::atomic_bool firstPoseUpdate;
		std::atomic_bool updateThreadLooping;
		std::thread updateThread;
		std::list<Update> updateList;
		std::mutex updateListLock;

        std::vector<std::shared_ptr<MapperModule>> mapperModuleVec;

		void updateThreadFunction();
		void applyUpdate(const Update& update);
		void loadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle);
		float toInferiorWorldCoordinate(const int& gridCoordinate) const;
		float toSuperiorWorldCoordinate(const int& gridCoordinate) const;
		void unloadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle);
		int toGridCoordinate(const float& worldCoordinate) const;
		int getMinGridCoordinate() const;
		int getMaxGridCoordinate() const;
		int toInferiorGridCoordinate(const float& worldCoordinate, const float& range) const;
		int toSuperiorGridCoordinate(const float& worldCoordinate, const float& range) const;
		void scheduleUpdate(const Update& update);
	public:
		Map(const bool& is3D, const bool& isOnline,
			const bool& saveCellsOnHardDrive, PM::ICPSequence& icp, std::mutex& icpMapLock);
		~Map();
		void updatePose(const PM::TransformationParameters& pose);
		PM::DataPoints getLocalPointCloud();
		void updateLocalPointCloud(PM::DataPoints input, PM::TransformationParameters pose, PM::DataPointsFilters postFilters);
		bool getNewLocalPointCloud(PM::DataPoints& localPointCloudOut);
		PM::DataPoints getGlobalPointCloud();
		void setGlobalPointCloud(const PM::DataPoints& newLocalPointCloud);
		inline bool isLocalPointCloudEmpty() const
        {
            return localPointCloudEmpty.load();
        }
        void addMapperModule(std::shared_ptr<MapperModule> mapperModule)
        {
            this->mapperModuleVec.push_back(std::move(mapperModule));
        }

        bool computesDynamicPoints() {
            return std::any_of(mapperModuleVec.begin(), mapperModuleVec.end(), [](const auto &module) {
                return module->className == "ComputeDynamicsMapperModule";
            });
        }

        void setSensorMaxRange(float inputSensorMaxRange) {
            this->sensorMaxRange = inputSensorMaxRange;
        }

        float getSensorMaxRange() {
            return this->sensorMaxRange;
        }
	};
}

#endif
