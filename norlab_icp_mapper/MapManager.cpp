#include "MapManager.h"
#include <fstream>
#include <dirent.h>
#include <sys/stat.h>

// Take is3D into acccout!

norlab_icp_mapper::MapManager::MapManager(const float& sensorMaxRange, std::mutex& mapLock, const bool& is3D, const bool& isOnline):
	sensorMaxRange(sensorMaxRange),
    mapLock(mapLock),
    is3D(is3D),
    isOnline(isOnline),
    firstLocalization(true)
{
    struct stat info;
    if(stat(CELL_FOLDER.c_str(), &info) != 0)
    {
        mkdir(CELL_FOLDER.c_str(), 0777);
    }

    std::vector<std::string> cellFiles = listCellFiles();
    for(int i = 0; i < cellFiles.size(); i++)
    {
        std::remove(cellFiles[i].c_str());
    }

    applyUpdatesThread = std::thread(&MapManager::applyUpdates, this);
    applyUpdatesThread.detach();
}

std::vector<std::string> norlab_icp_mapper::MapManager::listCellFiles()
{
    DIR* dir;
    struct dirent* diread;
    std::vector<std::string> cellFiles;
    if((dir = opendir(CELL_FOLDER.c_str())) != nullptr)
    {
        while((diread = readdir(dir)) != nullptr)
        {
            std::string fileName(diread->d_name);
            if(fileName.find("cell_") != std::string::npos)
            {
                cellFiles.push_back(CELL_FOLDER + fileName);
            }
        }
        closedir(dir);
    }
    return cellFiles;
}

int norlab_icp_mapper::MapManager::toGridCoordinate(const float& worldCoordinate)
{
    return std::floor(worldCoordinate / GRID_CELL_SIZE);
}

int norlab_icp_mapper::MapManager::toInferiorGridCoordinate(const float& worldCoordinate, const float& range)
{
    return std::ceil(((worldCoordinate - range) / GRID_CELL_SIZE) - 1.0);
}

int norlab_icp_mapper::MapManager::toSuperiorGridCoordinate(const float& worldCoordinate, const float& range)
{
    return std::floor((worldCoordinate + range) / GRID_CELL_SIZE);
}

void norlab_icp_mapper::MapManager::setCurrentPose(const PM::TransformationParameters& newPose)
{
    if(firstLocalization)
    {
        inferiorRowLastUpdateIndex = toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange);
        superiorRowLastUpdateIndex = toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange);
        inferiorColumnLastUpdateIndex = toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange);
        superiorColumnLastUpdateIndex = toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange);
        inferiorAisleLastUpdateIndex = toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange);
        superiorAisleLastUpdateIndex = toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange);

        firstLocalization = false;
    }
    else
    {
        // manage cells in the back
        if(std::abs(toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange) - inferiorRowLastUpdateIndex) >= 2)
        {
            // move back
            if(toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange) < inferiorRowLastUpdateIndex)
            {
                int nbRows = inferiorRowLastUpdateIndex - toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange);
                int startRow = toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange) - BUFFER_SIZE;
                int endRow = toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange) - BUFFER_SIZE + nbRows - 1;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
                updateListLock.unlock();
            }
            // move front
            if(toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange) > inferiorRowLastUpdateIndex)
            {
                int nbRows = toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange) - inferiorRowLastUpdateIndex;
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = inferiorRowLastUpdateIndex - BUFFER_SIZE + nbRows - 1;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
                updateListLock.unlock();
            }
            inferiorRowLastUpdateIndex = toInferiorGridCoordinate(newPose(0, 3), sensorMaxRange);
        }
        
        // manage cells in front
        if(std::abs(toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange) - superiorRowLastUpdateIndex) >= 2)
        {
            // move back
            if(toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange) < superiorRowLastUpdateIndex)
            {
                int nbRows = superiorRowLastUpdateIndex - toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange);
                int startRow = superiorRowLastUpdateIndex + BUFFER_SIZE - nbRows + 1;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
                updateListLock.unlock();
            }
            // move front
            if(toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange) > superiorRowLastUpdateIndex)
            {
                int nbRows = toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange) - superiorRowLastUpdateIndex;
                int startRow = toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange) + BUFFER_SIZE - nbRows + 1;
                int endRow = toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange) + BUFFER_SIZE;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
                updateListLock.unlock();
            }
            superiorRowLastUpdateIndex = toSuperiorGridCoordinate(newPose(0, 3), sensorMaxRange);
        }

        // update cells to the right
        if(std::abs(toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange) - inferiorColumnLastUpdateIndex) >= 2)
        {
            // move right
            if(toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange) < inferiorColumnLastUpdateIndex)
            {
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int nbColumns = inferiorColumnLastUpdateIndex - toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange);
                int startColumn = toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange) - BUFFER_SIZE;
                int endColumn = toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange) - BUFFER_SIZE + nbColumns - 1;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
                updateListLock.unlock();
            }
            // move left
            if(toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange) > inferiorColumnLastUpdateIndex)
            {
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int nbColumns = toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange) - inferiorColumnLastUpdateIndex;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE + nbColumns - 1;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
                updateListLock.unlock();
            }
            inferiorColumnLastUpdateIndex = toInferiorGridCoordinate(newPose(1, 3), sensorMaxRange);
        }

        // update cells to the left
        if(std::abs(toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange) - superiorColumnLastUpdateIndex) >= 2)
        {
            // move right
            if(toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange) < superiorColumnLastUpdateIndex)
            {
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int nbColumns = superiorColumnLastUpdateIndex - toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange);
                int startColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE - nbColumns + 1;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
                updateListLock.unlock();
            }
            // move left
            if(toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange) > superiorColumnLastUpdateIndex)
            {
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int nbColumns = toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange) - superiorColumnLastUpdateIndex;
                int startColumn = toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange) + BUFFER_SIZE - nbColumns + 1;
                int endColumn = toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange) + BUFFER_SIZE;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
                updateListLock.unlock();
            }
            superiorColumnLastUpdateIndex = toSuperiorGridCoordinate(newPose(1, 3), sensorMaxRange);
        }

        // update cells below
        if(std::abs(toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange) - inferiorAisleLastUpdateIndex) >= 2)
        {
            // move down
            if(toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange) < inferiorAisleLastUpdateIndex)
            {
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int nbAisles = inferiorAisleLastUpdateIndex - toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange);
                int startAisle = toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange) - BUFFER_SIZE;
                int endAisle = toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange) - BUFFER_SIZE + nbAisles - 1;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
                updateListLock.unlock();
            }
            // move up
            if(toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange) > inferiorAisleLastUpdateIndex)
            {
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int nbAisles = toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange) - inferiorAisleLastUpdateIndex;
                int startAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE;
                int endAisle = inferiorAisleLastUpdateIndex - BUFFER_SIZE + nbAisles - 1;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
                updateListLock.unlock();
            }
            inferiorAisleLastUpdateIndex = toInferiorGridCoordinate(newPose(2, 3), sensorMaxRange);
        }

        // update cells above
        if(std::abs(toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange) - superiorAisleLastUpdateIndex) >= 2)
        {
            // move down
            if(toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange) < superiorAisleLastUpdateIndex)
            {
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int nbAisles = superiorAisleLastUpdateIndex - toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange);
                int startAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE - nbAisles + 1;
                int endAisle = superiorAisleLastUpdateIndex + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, false});
                updateListLock.unlock();
            }
            // move up
            if(toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange) > superiorAisleLastUpdateIndex)
            {
                int startRow = inferiorRowLastUpdateIndex - BUFFER_SIZE;
                int endRow = superiorRowLastUpdateIndex + BUFFER_SIZE;
                int startColumn = inferiorColumnLastUpdateIndex - BUFFER_SIZE;
                int endColumn = superiorColumnLastUpdateIndex + BUFFER_SIZE;
                int nbAisles = toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange) - superiorAisleLastUpdateIndex;
                int startAisle = toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange) + BUFFER_SIZE - nbAisles + 1;
                int endAisle = toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange) + BUFFER_SIZE;
                updateListLock.lock();
                updateList.push_back(Update{startRow, endRow, startColumn, endColumn, startAisle, endAisle, true});
                updateListLock.unlock();
            }
            superiorAisleLastUpdateIndex = toSuperiorGridCoordinate(newPose(2, 3), sensorMaxRange);
        }
    }
}

void norlab_icp_mapper::MapManager::applyUpdates()
{
    while(true)
    {
        updateListLock.lock();
        bool isUpdateListEmpty = updateList.empty();
        updateListLock.unlock();
        if(!isUpdateListEmpty)
        {
            updateListLock.lock();
            Update update = updateList.front();
            updateListLock.unlock();
            if(update.load)
            {
                loadCells(update.startRow, update.endRow, update.startColumn, update.endColumn, update.startAisle, update.endAisle);
            }
            else
            {
                unloadCells(update.startRow, update.endRow, update.startColumn, update.endColumn, update.startAisle, update.endAisle);
            }
            updateListLock.lock();
            updateList.pop_front();
            updateListLock.unlock();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::duration<float>(0.01));
        }
    }
}

void norlab_icp_mapper::MapManager::loadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle)
{
    PM::DataPoints newChunk;
    for(int i = startRow; i <= endRow; i++)
    {
        for(int j = startColumn; j <= endColumn; j++)
        {
            for(int k = startAisle; k <= endAisle; k++)
            {
                std::string fileName = CELL_FOLDER + "cell_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k) + ".vtk";
                std::ifstream file(fileName.c_str());
                bool fileExists = file.good();
                file.close();
                if(fileExists)
                {
                    PM::DataPoints cell = PM::DataPoints::load(fileName);

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

    if(newChunk.getNbPoints() > 0)
    {
        mapLock.lock();
        map.concatenate(newChunk);
        newMapAvailable = true;
        mapLock.unlock();
    }
}

void norlab_icp_mapper::MapManager::unloadCells(int startRow, int endRow, int startColumn, int endColumn, int startAisle, int endAisle)
{
    int nbRows = endRow - startRow + 1;
    int nbColumns = endColumn - startColumn + 1;
    int nbAisles = endAisle - startAisle + 1;
    float startX = toInferiorWorldCoordinate(startRow);
    float endX = toSuperiorWorldCoordinate(endRow);
    float startY = toInferiorWorldCoordinate(startColumn);
    float endY = toSuperiorWorldCoordinate(endColumn);
    float startZ = toInferiorWorldCoordinate(startAisle);
    float endZ = toSuperiorWorldCoordinate(endAisle);

    int mapNbPoints = 0;
    int oldChunkNbPoints = 0;
    mapLock.lock();
    PM::DataPoints oldChunk = map.createSimilarEmpty();
    for(int i = 0; i < map.features.cols(); i++)
    {
        if(map.features(0, i) >= startX && map.features(0, i) < endX && map.features(1, i) >= startY && map.features(1, i) < endY && map.features(2, i) >= startZ && map.features(2, i) < endZ)
        {
            oldChunk.setColFrom(oldChunkNbPoints, map, i);
            oldChunkNbPoints++;
        }
        else
        {
            map.setColFrom(mapNbPoints, map, i);
            mapNbPoints++;
        }
    }
    map.conservativeResize(mapNbPoints);
    newMapAvailable = true;
    mapLock.unlock();
    oldChunk.conservativeResize(oldChunkNbPoints);

    std::vector<std::pair<std::tuple<int, int, int>, PM::DataPoints>> cells;
    std::vector<int> cellPointCounts;
    for(int i = 0; i < nbRows * nbColumns * nbAisles; i++)
    {
        cells.push_back(std::make_pair(std::make_tuple(0, 0, 0), oldChunk.createSimilarEmpty()));
        cellPointCounts.push_back(0);
    }
    for(int i = 0; i < oldChunk.getNbPoints(); i++)
    {
        int row = toGridCoordinate(oldChunk.features(0, i));
        int column = toGridCoordinate(oldChunk.features(1, i));
        int aisle = toGridCoordinate(oldChunk.features(2, i));
        int rowOffset = row - startRow;
        int columnOffset = column - startColumn;
        int aisleOffset = aisle - startAisle;
        int cellIndex = (rowOffset * nbColumns * nbAisles) + (columnOffset * nbAisles) + aisleOffset;

        std::get<0>(cells[cellIndex].first) = row;
        std::get<1>(cells[cellIndex].first) = column;
        std::get<2>(cells[cellIndex].first) = aisle;
        cells[cellIndex].second.setColFrom(cellPointCounts[cellIndex], oldChunk, i);
        cellPointCounts[cellIndex]++;
    }
    for(int i = 0; i < nbRows * nbColumns * nbAisles; i++)
    {
        if(cellPointCounts[i] > 0)
        {
            cells[i].second.conservativeResize(cellPointCounts[i]);
            int row = std::get<0>(cells[i].first);
            int column = std::get<1>(cells[i].first);
            int aisle = std::get<2>(cells[i].first);
            cells[i].second.save(CELL_FOLDER + "cell_" + std::to_string(row) + "_" + std::to_string(column) + "_" + std::to_string(aisle) + ".vtk");
        }
    }
}

float norlab_icp_mapper::MapManager::toInferiorWorldCoordinate(const int& gridCoordinate)
{
    return gridCoordinate * GRID_CELL_SIZE;
}

float norlab_icp_mapper::MapManager::toSuperiorWorldCoordinate(const int& gridCoordinate)
{
    return (gridCoordinate + 1) * GRID_CELL_SIZE;
}

norlab_icp_mapper::PM::DataPoints norlab_icp_mapper::MapManager::getLocalMap()
{
    return map;
}

void norlab_icp_mapper::MapManager::setLocalMap(const PM::DataPoints& newMap)
{
    map = newMap;
    newMapAvailable = true;
}

bool norlab_icp_mapper::MapManager::getNewLocalMap(PM::DataPoints& mapOut)
{
	bool mapReturned = false;

    if(newMapAvailable)
    {
        mapOut = map;
        newMapAvailable = false;
        mapReturned = true;
    }
    
    return mapReturned;
}

norlab_icp_mapper::PM::DataPoints norlab_icp_mapper::MapManager::getGlobalMap()
{
    // some points will be there twice because of the copy + load file
    mapLock.lock();
    PM::DataPoints globalMap = map;
    mapLock.unlock();
    std::vector<std::string> cellFiles = listCellFiles();
    for(int i = 0; i < cellFiles.size(); i++)
    {
        globalMap.concatenate(PM::DataPoints::load(cellFiles[i]));
    }
	return globalMap;
}

void norlab_icp_mapper::MapManager::setGlobalMap(const PM::DataPoints& newMap)
{
	setLocalMap(newMap);
}

