#ifndef RAM_CELL_MANAGER_H
#define RAM_CELL_MANAGER_H

#include "CellManager.h"

namespace norlab_icp_mapper
{
	class RAMCellManager : public CellManager
	{
	private:
		std::unordered_map<CellInfo, PM::DataPoints> cells;

	public:
		void saveCell(const CellInfo& cellInfo, const PM::DataPoints& cell) override;
		std::pair<CellInfo, PM::DataPoints> retrieveCell(const int& row, const int& column, const int& aisle, const int& depth) const override;
		void clearAllCells() override;
	};
}

#endif
