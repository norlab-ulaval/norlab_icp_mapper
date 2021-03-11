#include <pointmatcher/PointMatcher.h>
#include <thread>
#include <mutex>
#include <list>
#include "CellManager.h"

namespace norlab_icp_mapper
{
	typedef float T;
	typedef PointMatcher<T> PM;

	class Map
	{
	private:
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

		float sensorMaxRange;
		float minDistNewPoint;
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
		std::set<std::string> loadedCells;
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
		std::atomic_bool looping;
		std::thread updateLoopThread;
		std::list<Update> updateList;
		std::mutex updateListLock;

		void updateLoop();
		void loadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle);
		void unloadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle);
		int toInferiorGridCoordinate(const float& worldCoordinate, const float& range) const;
		int toSuperiorGridCoordinate(const float& worldCoordinate, const float& range) const;
		void scheduleUpdate(const Update& update);
		float toInferiorWorldCoordinate(const int& gridCoordinate) const;
		float toSuperiorWorldCoordinate(const int& gridCoordinate) const;
		int toGridCoordinate(const float& worldCoordinate) const;
		PM::DataPoints retrievePointsFurtherThanMinDistNewPoint(const PM::DataPoints& input, const PM::DataPoints& currentLocalPointCloud,
																const PM::TransformationParameters& pose) const;
		void computeProbabilityOfPointsBeingDynamic(const PM::DataPoints& input, PM::DataPoints& currentLocalPointCloud,
													const PM::TransformationParameters& pose) const;
		void convertToSphericalCoordinates(const PM::DataPoints& points, PM::Matrix& radii, PM::Matrix& angles) const;

	public:
		Map(const float& minDistNewPoint, const float& sensorMaxRange, const float& priorDynamic, const float& thresholdDynamic, const float& beamHalfAngle,
			const float& epsilonA, const float& epsilonD, const float& alpha, const float& beta, const bool& is3D, const bool& isOnline,
			const bool& computeProbDynamic, PM::ICPSequence& icp, std::mutex& icpMapLock);
		~Map();
		void updatePose(const PM::TransformationParameters& pose);
		PM::DataPoints getLocalPointCloud();
		void updateLocalPointCloud(PM::DataPoints input, PM::TransformationParameters pose, PM::DataPointsFilters postFilters);
		bool getNewLocalPointCloud(PM::DataPoints& localPointCloudOut);
		PM::DataPoints getGlobalPointCloud();
		void setGlobalPointCloud(const PM::DataPoints& newLocalPointCloud);
		bool isLocalPointCloudEmpty() const;
	};
}
