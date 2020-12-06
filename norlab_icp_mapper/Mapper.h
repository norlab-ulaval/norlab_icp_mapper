#include <pointmatcher/PointMatcher.h>
#include <future>

namespace norlab_icp_mapper
{
	typedef float T;
	typedef PointMatcher<T> PM;
	
	class Mapper
	{
	private:
		PM::DataPointsFilters inputFilters;
		PM::DataPointsFilters mapPostFilters;
		PM::ICPSequence icp;
		PM::DataPoints map;
		PM::TransformationParameters sensorPose;
		std::shared_ptr<PM::Transformation> transformation;
		std::shared_ptr<PM::DataPointsFilter> radiusFilter;
		std::chrono::time_point<std::chrono::steady_clock> lastTimeMapWasUpdated;
		PM::TransformationParameters lastSensorPoseWhereMapWasUpdated;
		std::string icpConfigFilePath;
		std::string inputFiltersConfigFilePath;
		std::string mapPostFiltersConfigFilePath;
		std::string mapUpdateCondition;
		float mapUpdateOverlap;
		float mapUpdateDelay;
		float mapUpdateDistance;
		float minDistNewPoint;
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
		bool useSkewWeights;
		bool isMapping;
		int skewModel;
		float cornerPointWeight;
		float weightQuantile;
		float rangePrecision;
		bool newMapAvailable;
		std::atomic_bool isMapEmpty;
		std::mutex mapLock;
		std::mutex icpMapLock;
		std::future<void> mapBuilderFuture;
		
		bool shouldUpdateMap(const std::chrono::time_point<std::chrono::steady_clock>& currentTime, const PM::TransformationParameters& currentSensorPose,
							 const float& currentOverlap);
		
		void updateMap(const PM::DataPoints& currentInput, const std::chrono::time_point<std::chrono::steady_clock>& timeStamp);
		
		void buildMap(PM::DataPoints currentInput, PM::DataPoints currentMap, PM::TransformationParameters currentSensorPose);
		
		PM::DataPoints retrievePointsFurtherThanMinDistNewPoint(const PM::DataPoints& currentInput, const PM::DataPoints& currentMap,
																const PM::TransformationParameters& currentSensorPose);
		
		void computeProbabilityOfPointsBeingDynamic(const PM::DataPoints& currentInput, PM::DataPoints& currentMap,
													const PM::TransformationParameters& currentSensorPose);
		
		void convertToSphericalCoordinates(const PM::DataPoints& points, PM::Matrix& radii, PM::Matrix& angles);
	
	public:
		Mapper(std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string mapPostFiltersConfigFilePath, std::string mapUpdateCondition,
			   float mapUpdateOverlap, float mapUpdateDelay, float mapUpdateDistance, float minDistNewPoint, float sensorMaxRange,
			   float priorDynamic, float thresholdDynamic, float beamHalfAngle, float epsilonA, float epsilonD, float alpha, float beta,
			   bool is3D, bool isOnline, bool computeProbDynamic, bool useSkewWeights, bool isMapping, int skewModel, float cornerPointWeight,
			   float weightQuantile, float rangePrecision);
		
		void loadYamlConfig();
		
		void processInput(PM::DataPoints& inputInSensorFrame, const PM::TransformationParameters& estimatedSensorPose,
						  const std::chrono::time_point<std::chrono::steady_clock>& timeStamp, const std::string& linearSpeedNoisesX,
						  const std::string& linearSpeedNoisesY, const std::string& linearSpeedNoisesZ, const std::string& linearAccelerationNoisesX,
						  const std::string& linearAccelerationNoisesY, const std::string& linearAccelerationNoisesZ,
						  const std::string& angularSpeedNoisesX, const std::string& angularSpeedNoisesY, const std::string& angularSpeedNoisesZ,
						  const std::string& angularAccelerationNoisesX, const std::string& angularAccelerationNoisesY,
						  const std::string& angularAccelerationNoisesZ, const std::string& measureTimes);
		
		PM::DataPoints getMap();
		
		void setMap(const PM::DataPoints& newMap, const PM::TransformationParameters& newSensorPose);
		
		bool getNewMap(PM::DataPoints& mapOut);
		
		const PM::TransformationParameters& getSensorPose();
	};
}
