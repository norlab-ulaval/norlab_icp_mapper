#include "RAMCellManager.h"

void norlab_icp_mapper::RAMCellManager::saveCell(const CellInfo& cellInfo, const PM::DataPoints& cell)
{
	cellInfos.insert(cellInfo);
	cells[cellInfo] = cell;
}

std::pair<norlab_icp_mapper::CellInfo, norlab_icp_mapper::CellManager::PM::DataPoints> norlab_icp_mapper::RAMCellManager::retrieveCell(const int& row, const int& column,
																																	   const int& aisle, const int& depth) const
{
	CellInfo cellInfo = computeInfoOfCellToRetrieve(row, column, aisle, depth);
	PM::DataPoints cell;
	if(cellInfo.depth != INVALID_CELL_DEPTH)
	{
		cell = cells.at(cellInfo);
	}
	return std::make_pair(cellInfo, cell);
}

void norlab_icp_mapper::RAMCellManager::clearAllCells()
{
	cellInfos.clear();
	cells.clear();
}
