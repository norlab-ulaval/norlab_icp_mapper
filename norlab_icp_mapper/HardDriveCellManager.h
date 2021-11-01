#ifndef HARD_DRIVE_CELL_MANAGER_H
#define HARD_DRIVE_CELL_MANAGER_H

#include "CellManager.h"

namespace norlab_icp_mapper
{
	class HardDriveCellManager : public CellManager
	{
	private:
		const std::string CELL_FOLDER = "/tmp/";
		const std::string CELL_FILE_NAME_PREFIX = "cell_";
		const std::string CELL_FILE_NAME_SUFFIX = ".vtk";

	public:
		~HardDriveCellManager() override;
		void saveCell(const CellInfo& cellInfo, const PM::DataPoints& cell) override;
		std::pair<CellInfo, PM::DataPoints> retrieveCell(const int& row, const int& column, const int& aisle, const int& depth) const override;
		void clearAllCells() override;
	};
}

#endif
