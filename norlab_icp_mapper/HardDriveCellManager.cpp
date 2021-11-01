#include "HardDriveCellManager.h"
#include <fstream>

norlab_icp_mapper::HardDriveCellManager::~HardDriveCellManager()
{
	clearAllCells();
}

void norlab_icp_mapper::HardDriveCellManager::saveCell(const CellInfo& cellInfo, const PM::DataPoints& cell)
{
	cellInfos.insert(cellInfo);
	cell.save(CELL_FOLDER + CELL_FILE_NAME_PREFIX + cellInfo.getName() + CELL_FILE_NAME_SUFFIX);
}

std::pair<norlab_icp_mapper::CellInfo, norlab_icp_mapper::CellManager::PM::DataPoints> norlab_icp_mapper::HardDriveCellManager::retrieveCell(const int& row, const int& column,
																																			 const int& aisle,
																																			 const int& depth) const
{
	CellInfo cellInfo = computeInfoOfCellToRetrieve(row, column, aisle, depth);
	PM::DataPoints cell;
	if(cellInfo.depth != INVALID_CELL_DEPTH)
	{
		cell = PM::DataPoints::load(CELL_FOLDER + CELL_FILE_NAME_PREFIX + cellInfo.getName() + CELL_FILE_NAME_SUFFIX);
	}
	return std::make_pair(cellInfo, cell);
}

void norlab_icp_mapper::HardDriveCellManager::clearAllCells()
{
	cellInfos.clear();
	for(const auto& cellInfo: cellInfos)
	{
		std::remove((CELL_FOLDER + CELL_FILE_NAME_PREFIX + cellInfo.getName() + CELL_FILE_NAME_SUFFIX).c_str());
	}
}
