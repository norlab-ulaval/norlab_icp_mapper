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

		float sensorMaxRange;
		float priorDynamic;
		float thresholdDynamic;
		float beamHalfAngle;
		float epsilonA;
		float epsilonD;
		float alpha;
		float beta;
		bool is3D;
		bool isOnline;
		bool computeProbDynamic;
		PM::ICPSequence& icp;
		std::mutex& icpMapLock;
		PM::DataPoints localPointCloud;
		std::mutex localPointCloudLock;
		std::unique_ptr<CellManager> cellManager;
		std::mutex cellManagerLock;
		std::unordered_set<std::string> loadedCellIds;
		std::shared_ptr<PM::Transformation> transformation;
		int inferiorRowLastUpdateIndex;
		int superiorRowLastUpdateIndex;
		int inferiorColumnLastUpdateIndex;
		int superiorColumnLastUpdateIndex;
		int inferiorAisleLastUpdateIndex;
		int superiorAisleLastUpdateIndex;
		bool newLocalPointCloudAvailable;
		std::atomic_bool localPointCloudEmpty;
		std::atomic_bool firstPoseUpdate;
		std::atomic_bool updateThreadLooping;
		std::thread updateThread;
		std::list<Update> updateList;
		std::mutex updateListLock;

        std::shared_ptr<MapperModule> mappingModule;

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
		void computeProbabilityOfPointsBeingDynamic(const PM::DataPoints& input, PM::DataPoints& currentLocalPointCloud,
													const PM::TransformationParameters& pose) const;
		void convertToSphericalCoordinates(const PM::DataPoints& points, PM::Matrix& radii, PM::Matrix& angles) const;

	public:
		Map(const float& sensorMaxRange, const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle,
			const float& epsilonA, const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& isOnline,
			const bool& computeProbDynamic, const bool& saveCellsOnHardDrive, PM::ICPSequence& icp, std::mutex& icpMapLock);
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
        void setMappingModule(std::shared_ptr<MapperModule> mappingModule)
        {
            this->mappingModule = std::move(mappingModule);
        }
	};
}

#endif
