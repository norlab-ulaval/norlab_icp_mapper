#include <pointmatcher/PointMatcher.h>
#include <thread>
#include <mutex>
#include <list>

namespace norlab_icp_mapper
{
	typedef float T;
	typedef PointMatcher<T> PM;
	
	class MapManager
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

		const float GRID_CELL_SIZE = 20.0;
		const int BUFFER_SIZE = 2;
		const std::string CELL_FOLDER = "/tmp/";

		PM::DataPoints map;
		float sensorMaxRange;
		bool is3D;
		bool isOnline;
		bool newMapAvailable;
		bool firstLocalization;
		std::mutex& mapLock;
		std::thread applyUpdatesThread;
		std::list<Update> updateList;
		std::mutex updateListLock;
		int inferiorRowLastUpdateIndex;
		int superiorRowLastUpdateIndex;
		int inferiorColumnLastUpdateIndex;
		int superiorColumnLastUpdateIndex;
		int inferiorAisleLastUpdateIndex;
		int superiorAisleLastUpdateIndex;

		int toGridCoordinate(const float& worldCoordinate);

		int toInferiorGridCoordinate(const float& worldCoordinate, const float& range);

		int toSuperiorGridCoordinate(const float& worldCoordinate, const float& range);

		float toInferiorWorldCoordinate(const int& gridCoordinate);

		float toSuperiorWorldCoordinate(const int& gridCoordinate);
		
		void applyUpdates();

		void loadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle);
		
		void unloadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle);

		std::vector<std::string> listCellFiles();
		
	public:
		MapManager(const float& sensorMaxRange, std::mutex& mapLock, const bool& is3D, const bool& isOnline);

		void setCurrentPose(const PM::TransformationParameters& newPose);
		
		PM::DataPoints getLocalMap();
		
		void setLocalMap(const PM::DataPoints& newMap);
		
		bool getNewLocalMap(PM::DataPoints& mapOut);

		void setGlobalMap(const PM::DataPoints& newMap);

		PM::DataPoints getGlobalMap();
	};
}

