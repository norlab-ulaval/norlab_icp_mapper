#ifndef HARD_DRIVE_CELL_MANAGER_H
#define HARD_DRIVE_CELL_MANAGER_H

#include "CellManager.h"
#include <unordered_set>

namespace norlab_icp_mapper
{
	class HardDriveCellManager : public CellManager
	{
	private:
		const std::string CELL_FOLDER = "/tmp/";
		const std::string CELL_FILE_NAME_PREFIX = "cell_";
		const std::string CELL_FILE_NAME_SUFFIX = ".vtk";
		std::unordered_set<std::string> cellIds;

	public:
		~HardDriveCellManager() override;
		std::vector<std::string> getAllCellIds() const override;
		void saveCell(const std::string& cellId, const PM::DataPoints& cell) override;
		PM::DataPoints retrieveCell(const std::string& cellId) const override;
		void clearAllCells() override;
	};
}

#endif
