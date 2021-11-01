#ifndef CELL_MANAGER_H
#define CELL_MANAGER_H

#include <pointmatcher/PointMatcher.h>
#include <unordered_set>
#include <unordered_map>
#include "CellInfo.h"

namespace norlab_icp_mapper
{
	class CellManager
	{
	protected:
		typedef PointMatcher<float> PM;
		const int DEPTH_DIFFERENCE_THRESHOLD = 2;

		CellInfo computeInfoOfCellToRetrieve(const int& row, const int& column, const int& aisle, const int& depth) const;

		std::unordered_set<CellInfo> cellInfos;

	public:
		static const int INVALID_CELL_DEPTH = -1;

		virtual ~CellManager() = default;
		virtual std::unordered_set<CellInfo> getAllCellInfos() const;
		virtual void saveCell(const CellInfo& cellInfo, const PM::DataPoints& cell) = 0;
		virtual std::pair<CellInfo, PM::DataPoints> retrieveCell(const int& row, const int& column, const int& aisle, const int& depth) const = 0;
		virtual void clearAllCells() = 0;
	};
}

#endif
