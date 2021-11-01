#include "CellManager.h"

std::unordered_set<norlab_icp_mapper::CellInfo> norlab_icp_mapper::CellManager::getAllCellInfos() const
{
	return cellInfos;
}

norlab_icp_mapper::CellInfo norlab_icp_mapper::CellManager::computeInfoOfCellToRetrieve(const int& row, const int& column, const int& aisle, const int& depth) const
{
	int minimumDepthDifference = DEPTH_DIFFERENCE_THRESHOLD;
	int closestCellDepth = std::numeric_limits<int>::max();
	for(int cellDepth = depth - DEPTH_DIFFERENCE_THRESHOLD; cellDepth <= depth + DEPTH_DIFFERENCE_THRESHOLD; ++cellDepth)
	{
		if(cellInfos.find(CellInfo(row, column, aisle, cellDepth)) != cellInfos.end())
		{
			int depthDifference = std::abs(depth - cellDepth);
			if(depthDifference < minimumDepthDifference || (depthDifference == minimumDepthDifference && cellDepth < closestCellDepth))
			{
				minimumDepthDifference = depthDifference;
				closestCellDepth = cellDepth;
			}
		}
	}
	int depthOfCellToRetrieve = closestCellDepth == std::numeric_limits<int>::max() ? INVALID_CELL_DEPTH : closestCellDepth;
	return CellInfo(row, column, aisle, depthOfCellToRetrieve);
}
