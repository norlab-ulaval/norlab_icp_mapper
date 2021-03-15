#include "HardDriveCellManager.h"
#include <fstream>

norlab_icp_mapper::HardDriveCellManager::~HardDriveCellManager()
{
	clearAllCells();
}

std::vector<std::string> norlab_icp_mapper::HardDriveCellManager::getAllCellIds() const
{
	return std::vector<std::string>(cellIds.begin(), cellIds.end());
}

void norlab_icp_mapper::HardDriveCellManager::saveCell(const std::string& cellId, const PM::DataPoints& cell)
{
	cell.save(CELL_FOLDER + CELL_FILE_NAME_PREFIX + cellId + CELL_FILE_NAME_SUFFIX);
	cellIds.insert(cellId);
}

norlab_icp_mapper::PM::DataPoints norlab_icp_mapper::HardDriveCellManager::retrieveCell(const std::string& cellId) const
{
	PM::DataPoints cell;
	if(cellIds.find(cellId) != cellIds.end())
	{
		cell = PM::DataPoints::load(CELL_FOLDER + CELL_FILE_NAME_PREFIX + cellId + CELL_FILE_NAME_SUFFIX);
	}
	return cell;
}

void norlab_icp_mapper::HardDriveCellManager::clearAllCells()
{
	for(const auto& cellId: cellIds)
	{
		std::remove((CELL_FOLDER + CELL_FILE_NAME_PREFIX + cellId + CELL_FILE_NAME_SUFFIX).c_str());
	}
	cellIds.clear();
}
