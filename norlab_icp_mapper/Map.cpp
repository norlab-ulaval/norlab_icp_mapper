#include "Map.h"
#include "RAMCellManager.h"
#include "HardDriveCellManager.h"
#include <nabo/nabo.h>
#include <unordered_map>

norlab_icp_mapper::Map::Map(const bool& is3D,
							const bool& isOnline, const bool& saveCellsOnHardDrive, PM::ICPSequence& icp,
							std::mutex& icpMapLock):
		is3D(is3D),
		isOnline(isOnline),
		icp(icp),
		icpMapLock(icpMapLock),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation")),
		newLocalPointCloudAvailable(false),
		localPointCloudEmpty(true),
		firstPoseUpdate(true),
		updateThreadLooping(true)
{
	if(saveCellsOnHardDrive)
	{
		cellManager = std::unique_ptr<CellManager>(new HardDriveCellManager());
	}
	else
	{
		cellManager = std::unique_ptr<CellManager>(new RAMCellManager());
	}

	if(isOnline)
	{
		updateThread = std::thread(&Map::updateThreadFunction, this);
	}
}

void norlab_icp_mapper::Map::updateThreadFunction()
{
	while(updateThreadLooping.load())
	{
		updateListLock.lock();
		bool isUpdateListEmpty = updateList.empty();
		updateListLock.unlock();

		if(!isUpdateListEmpty)
		{
			updateListLock.lock();
			Update update = updateList.front();
			updateList.pop_front();
			updateListLock.unlock();

			applyUpdate(update);
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::duration<float>(0.01));
		}
	}
}

void norlab_icp_mapper::Map::applyUpdate(const Update& update)
{
	if(update.load)
	{
		loadCells(update.startRow, update.endRow, update.startColumn, update.endColumn, update.startAisle, update.endAisle);
	}
	else
	{
		unloadCells(update.startRow, update.endRow, update.startColumn, update.endColumn, update.startAisle, update.endAisle);
	}
}

void norlab_icp_mapper::Map::loadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle)
{
	if(!is3D)
	{
		startAisle = 0;
		endAisle = 0;
	}

	PM::DataPoints newChunk;
	for(int i = startRow; i <= endRow; i++)
	{
		for(int j = startColumn; j <= endColumn; j++)
		{
			for(int k = startAisle; k <= endAisle; k++)
			{
				cellManagerLock.lock();
				PM::DataPoints cell = cellManager->retrieveCell(std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k));
				cellManagerLock.unlock();

				if(cell.getNbPoints() > 0)
				{
					if(newChunk.getNbPoints() == 0)
					{
						newChunk = cell;
					}
					else
					{
						newChunk.concatenate(cell);
					}
				}
			}
		}
	}

	localPointCloudLock.lock();
	if(newChunk.getNbPoints() > 0)
	{
		localPointCloud.concatenate(newChunk);

		icpMapLock.lock();
		icp.setMap(localPointCloud);
		icpMapLock.unlock();

		localPointCloudEmpty.store(false);
		newLocalPointCloudAvailable = true;
	}
	for(int i = startRow; i <= endRow; i++)
	{
		for(int j = startColumn; j <= endColumn; j++)
		{
			for(int k = startAisle; k <= endAisle; k++)
			{
				loadedCellIds.insert(std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k));
			}
		}
	}
	localPointCloudLock.unlock();
}

float norlab_icp_mapper::Map::toInferiorWorldCoordinate(const int& gridCoordinate) const
{
	return gridCoordinate * CELL_SIZE;
}

float norlab_icp_mapper::Map::toSuperiorWorldCoordinate(const int& gridCoordinate) const
{
	return (gridCoordinate + 1) * CELL_SIZE;
}

void norlab_icp_mapper::Map::unloadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle)
{
	if(!is3D)
	{
		startAisle = 0;
		endAisle = 0;
	}

	float startX = toInferiorWorldCoordinate(startRow);
	float endX = toSuperiorWorldCoordinate(endRow);
	float startY = toInferiorWorldCoordinate(startColumn);
	float endY = toSuperiorWorldCoordinate(endColumn);
	float startZ = toInferiorWorldCoordinate(startAisle);
	float endZ = toSuperiorWorldCoordinate(endAisle);

	int localPointCloudNbPoints = 0;
	int oldChunkNbPoints = 0;

	localPointCloudLock.lock();

	PM::DataPoints oldChunk = localPointCloud.createSimilarEmpty();
	for(int i = 0; i < localPointCloud.features.cols(); i++)
	{
		if(localPointCloud.features(0, i) >= startX && localPointCloud.features(0, i) < endX && localPointCloud.features(1, i) >= startY &&
		   localPointCloud.features(1, i) < endY && localPointCloud.features(2, i) >= startZ && localPointCloud.features(2, i) < endZ)
		{
			oldChunk.setColFrom(oldChunkNbPoints, localPointCloud, i);
			oldChunkNbPoints++;
		}
		else
		{
			localPointCloud.setColFrom(localPointCloudNbPoints, localPointCloud, i);
			localPointCloudNbPoints++;
		}
	}
	localPointCloud.conservativeResize(localPointCloudNbPoints);

	icpMapLock.lock();
	icp.setMap(localPointCloud);
	icpMapLock.unlock();

	if(!loadedCellIds.empty())
	{
		for(int i = startRow; i <= endRow; i++)
		{
			for(int j = startColumn; j <= endColumn; j++)
			{
				for(int k = startAisle; k <= endAisle; k++)
				{
					loadedCellIds.erase(std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k));
				}
			}
		}
	}

	localPointCloudEmpty.store(localPointCloud.getNbPoints() == 0);
	newLocalPointCloudAvailable = true;

	localPointCloudLock.unlock();

	oldChunk.conservativeResize(oldChunkNbPoints);

	std::unordered_map<std::string, PM::DataPoints> cells;
	std::unordered_map<std::string, int> cellPointCounts;
	for(int i = 0; i < oldChunk.getNbPoints(); i++)
	{
		int row = toGridCoordinate(oldChunk.features(0, i));
		int column = toGridCoordinate(oldChunk.features(1, i));
		int aisle = toGridCoordinate(oldChunk.features(2, i));
		std::string cellId = std::to_string(row) + "_" + std::to_string(column) + "_" + std::to_string(aisle);

		if(cells[cellId].getNbPoints() == 0)
        {
            cells[cellId] = oldChunk.createSimilarEmpty(INITIAL_CELL_NB_POINTS_WHEN_UNLOADING);
        }
        else if(cells[cellId].getNbPoints() == cellPointCounts[cellId])
        {
            cells[cellId].conservativeResize(cells[cellId].getNbPoints() * 2);
        }

		cells[cellId].setColFrom(cellPointCounts[cellId], oldChunk, i);
		cellPointCounts[cellId]++;
	}
	for(auto& cell: cells)
	{
		cell.second.conservativeResize(cellPointCounts[cell.first]);
		cellManagerLock.lock();
		cellManager->saveCell(cell.first, cell.second);
		cellManagerLock.unlock();
	}
}

int norlab_icp_mapper::Map::toGridCoordinate(const float& worldCoordinate) const
{
	return std::floor(worldCoordinate / CELL_SIZE);
}

norlab_icp_mapper::Map::~Map()
{
	if(isOnline)
	{
		updateThreadLooping.store(false);
		updateThread.join();
	}
}

void norlab_icp_mapper::Map::updatePose(const PM::TransformationParameters& pose)
{
	int positionColumn = is3D ? 3 : 2;
	if(firstPoseUpdate.load())
	{
		inferiorRowLastUpdateIndex = toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange);
		superiorRowLastUpdateIndex = toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange);
		inferiorColumnLastUpdateIndex = toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange);
		superiorColumnLastUpdateIndex = toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange);
		if(is3D)
		{
			inferiorAisleLastUpdateIndex = toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange);
			superiorAisleLastUpdateIndex = toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange);
		}

		cellManagerLock.lock();
		cellManager->clearAllCells();
		cellManagerLock.unlock();
		localPointCloudLock.lock();
		loadedCellIds.clear();
		localPointCloudLock.unlock();

		unloadCells(getMinGridCoordinate(), getMaxGridCoordinate(), getMinGridCoordinate(),
					getMaxGridCoordinate(), getMinGridCoordinate(), getMaxGridCoordinate());
		loadCells(inferiorRowLastUpdateIndex - BUFFER_SIZE, superiorRowLastUpdateIndex + BUFFER_SIZE, inferiorColumnLastUpdateIndex - BUFFER_SIZE,
				  superiorColumnLastUpdateIndex + BUFFER_SIZE, inferiorAisleLastUpdateIndex - BUFFER_SIZE, superiorAisleLastUpdateIndex + BUFFER_SIZE);

		firstPoseUpdate.store(false);
	}
	else
	{
		// manage cells in the back
		if(std::abs(toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) - inferiorRowLastUpdateIndex) >= 2)
		{
			// move back
			if(toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) < inferiorRowLastUpdateIndex)
			{
				int nbRows = inferiorRowLastUpdateIndex - toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange);
				int startRow = toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) - BUFFER_SIZE;
				int endRow = toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) - BUFFER_SIZE + nbRows - 1;
				int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
				int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
				int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
				int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
				scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
			}
			// move front
			if(toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) > inferiorRowLastUpdateIndex)
			{
				int nbRows = toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) - inferiorRowLastUpdateIndex;
				int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
				int endRow = inferiorRowLastUpdateIndex - BUFFER_SIZE + nbRows - 1;
				int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
				int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
				int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
				int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
				scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
			}
			inferiorRowLastUpdateIndex = toInferiorGridCoordinate(pose(0, positionColumn), sensorMaxRange);
		}

		// manage cells in front
		if(std::abs(toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) - superiorRowLastUpdateIndex) >= 2)
		{
			// move back
			if(toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) < superiorRowLastUpdateIndex)
			{
				int nbRows = superiorRowLastUpdateIndex - toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange);
				int startRow = superiorRowLastUpdateIndex + BUFFER_SIZE - nbRows + 1;
				int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
				int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
				int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
				int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
				int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
				scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
			}
			// move front
			if(toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) > superiorRowLastUpdateIndex)
			{
				int nbRows = toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) - superiorRowLastUpdateIndex;
				int startRow = toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) + BUFFER_SIZE - nbRows + 1;
				int endRow = toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange) + BUFFER_SIZE;
				int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
				int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
				int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
				int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
				scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
			}
			superiorRowLastUpdateIndex = toSuperiorGridCoordinate(pose(0, positionColumn), sensorMaxRange);
		}

		// update cells to the right
		if(std::abs(toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) - inferiorColumnLastUpdateIndex) >= 2)
		{
			// move right
			if(toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) < inferiorColumnLastUpdateIndex)
			{
				int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
				int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
				int nbColumns = inferiorColumnLastUpdateIndex - toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange);
				int startColumn = toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) - BUFFER_SIZE;
				int endColumn = toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) - BUFFER_SIZE + nbColumns - 1;
				int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
				int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
				scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
			}
			// move left
			if(toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) > inferiorColumnLastUpdateIndex)
			{
				int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
				int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
				int nbColumns = toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) - inferiorColumnLastUpdateIndex;
				int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
				int endColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE + nbColumns - 1;
				int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
				int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
				scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
			}
			inferiorColumnLastUpdateIndex = toInferiorGridCoordinate(pose(1, positionColumn), sensorMaxRange);
		}

		// update cells to the left
		if(std::abs(toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) - superiorColumnLastUpdateIndex) >= 2)
		{
			// move right
			if(toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) < superiorColumnLastUpdateIndex)
			{
				int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
				int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
				int nbColumns = superiorColumnLastUpdateIndex - toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange);
				int startColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE - nbColumns + 1;
				int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
				int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
				int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
				scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
			}
			// move left
			if(toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) > superiorColumnLastUpdateIndex)
			{
				int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
				int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
				int nbColumns = toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) - superiorColumnLastUpdateIndex;
				int startColumn = toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) + BUFFER_SIZE - nbColumns + 1;
				int endColumn = toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange) + BUFFER_SIZE;
				int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
				int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
				scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
			}
			superiorColumnLastUpdateIndex = toSuperiorGridCoordinate(pose(1, positionColumn), sensorMaxRange);
		}

		if(is3D)
		{
			// update cells below
			if(std::abs(toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) - inferiorAisleLastUpdateIndex) >= 2)
			{
				// move down
				if(toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) < inferiorAisleLastUpdateIndex)
				{
					int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
					int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
					int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
					int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
					int nbAisles = inferiorAisleLastUpdateIndex - toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange);
					int startAisle = toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) - BUFFER_SIZE;
					int endAisle = toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) - BUFFER_SIZE + nbAisles - 1;
					scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
				}
				// move up
				if(toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) > inferiorAisleLastUpdateIndex)
				{
					int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
					int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
					int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
					int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
					int nbAisles = toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) - inferiorAisleLastUpdateIndex;
					int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
					int endAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE + nbAisles - 1;
					scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
				}
				inferiorAisleLastUpdateIndex = toInferiorGridCoordinate(pose(2, positionColumn), sensorMaxRange);
			}

			// update cells above
			if(std::abs(toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) - superiorAisleLastUpdateIndex) >= 2)
			{
				// move down
				if(toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) < superiorAisleLastUpdateIndex)
				{
					int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
					int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
					int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
					int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
					int nbAisles = superiorAisleLastUpdateIndex - toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange);
					int startAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE - nbAisles + 1;
					int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
					scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
				}
				// move up
				if(toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) > superiorAisleLastUpdateIndex)
				{
					int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
					int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
					int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
					int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
					int nbAisles = toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) - superiorAisleLastUpdateIndex;
					int startAisle = toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) + BUFFER_SIZE - nbAisles + 1;
					int endAisle = toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange) + BUFFER_SIZE;
					scheduleUpdate(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
				}
				superiorAisleLastUpdateIndex = toSuperiorGridCoordinate(pose(2, positionColumn), sensorMaxRange);
			}
		}
	}
}

int norlab_icp_mapper::Map::getMinGridCoordinate() const
{
	return std::numeric_limits<int>::lowest();
}

int norlab_icp_mapper::Map::getMaxGridCoordinate() const
{
	return std::numeric_limits<int>::max() - 1;
}

int norlab_icp_mapper::Map::toInferiorGridCoordinate(const float& worldCoordinate, const float& range) const
{
	return std::ceil(((worldCoordinate - range) / CELL_SIZE) - 1.0);
}

int norlab_icp_mapper::Map::toSuperiorGridCoordinate(const float& worldCoordinate, const float& range) const
{
	return std::floor((worldCoordinate + range) / CELL_SIZE);
}

void norlab_icp_mapper::Map::scheduleUpdate(const Update& update)
{
	if(isOnline)
	{
		updateListLock.lock();
		updateList.push_back(update);
		updateListLock.unlock();
	}
	else
	{
		applyUpdate(update);
	}
}

norlab_icp_mapper::Map::PM::DataPoints norlab_icp_mapper::Map::getLocalPointCloud()
{
	std::lock_guard<std::mutex> lock(localPointCloudLock);
	return localPointCloud;
}

void norlab_icp_mapper::Map::updateLocalPointCloud(PM::DataPoints input, PM::TransformationParameters pose, PM::DataPointsFilters postFilters)
{
	localPointCloudLock.lock();
	if(isLocalPointCloudEmpty())
	{
        // create map with the first mapper module and use the other modules to update it
        auto iter = mapperModuleVec.begin();
        localPointCloud = (*iter)->createMap(input, pose);
        ++iter;

        for (; iter != mapperModuleVec.end(); ++iter) {
            (*iter)->inPlaceUpdateMap(input, localPointCloud, pose);
        }
	}
	else
	{
        for (const auto& module:mapperModuleVec) {
            module->inPlaceUpdateMap(input, localPointCloud, pose);
        }
	}

	PM::DataPoints localPointCloudInSensorFrame = transformation->compute(localPointCloud, pose.inverse());
	postFilters.apply(localPointCloudInSensorFrame);
	localPointCloud = transformation->compute(localPointCloudInSensorFrame, pose);

	icpMapLock.lock();
	icp.setMap(localPointCloud);
	icpMapLock.unlock();

	localPointCloudEmpty.store(localPointCloud.getNbPoints() == 0);
	newLocalPointCloudAvailable = true;
	localPointCloudLock.unlock();
}

bool norlab_icp_mapper::Map::getNewLocalPointCloud(PM::DataPoints& localPointCloudOut)
{
	bool localPointCloudReturned = false;

	localPointCloudLock.lock();
	if(newLocalPointCloudAvailable)
	{
		localPointCloudOut = localPointCloud;
		newLocalPointCloudAvailable = false;
		localPointCloudReturned = true;
	}
	localPointCloudLock.unlock();

	return localPointCloudReturned;
}

norlab_icp_mapper::Map::PM::DataPoints norlab_icp_mapper::Map::getGlobalPointCloud()
{
	localPointCloudLock.lock();
	PM::DataPoints globalMap = localPointCloud;
	std::unordered_set<std::string> currentLoadedCellIds = loadedCellIds;
	localPointCloudLock.unlock();
	cellManagerLock.lock();
	std::vector<std::string> savedCellIds = cellManager->getAllCellIds();
	cellManagerLock.unlock();

	for(const auto& savedCellId: savedCellIds)
	{
		if(currentLoadedCellIds.find(savedCellId) == currentLoadedCellIds.end())
		{
			cellManagerLock.lock();
			PM::DataPoints cell = cellManager->retrieveCell(savedCellId);
			cellManagerLock.unlock();
			globalMap.concatenate(cell);
		}
	}
	return globalMap;
}

void norlab_icp_mapper::Map::setGlobalPointCloud(const PM::DataPoints& newLocalPointCloud)
{
	if(computesDynamicPoints() && !newLocalPointCloud.descriptorExists("normals"))
	{
		throw std::runtime_error("compute prob dynamic is set to true, but field normals does not exist for map points.");
	}

	localPointCloudLock.lock();
	localPointCloud = newLocalPointCloud;

	icpMapLock.lock();
	icp.setMap(localPointCloud);
	icpMapLock.unlock();

	localPointCloudEmpty.store(localPointCloud.getNbPoints() == 0);

	firstPoseUpdate.store(true);
	localPointCloudLock.unlock();
}
