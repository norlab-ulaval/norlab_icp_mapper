#include "HardDriveCellManager.h"
#include <dirent.h>

std::vector<std::string> norlab_icp_mapper::HardDriveCellManager::getAllCellIds() const
{
	DIR* dir;
	struct dirent* diread;
	std::vector<std::string> cellIds;
	if((dir = opendir(CELL_FOLDER.c_str())) != nullptr)
	{
		while((diread = readdir(dir)) != nullptr)
		{
			std::string fileName(diread->d_name);
			if(fileName.find(CELL_FILE_NAME_PREFIX) != std::string::npos)
			{
				cellIds.push_back(fileName.substr(CELL_FILE_NAME_PREFIX.size(),
													fileName.size() - (CELL_FILE_NAME_PREFIX.size() + CELL_FILE_NAME_SUFFIX.size())));
				std::cout << fileName.substr(CELL_FILE_NAME_PREFIX.size(), fileName.size() - (CELL_FILE_NAME_PREFIX.size() + CELL_FILE_NAME_SUFFIX.size()))
						  << std::endl;
			}
		}
		closedir(dir);
	}
	return cellIds;
}

void norlab_icp_mapper::HardDriveCellManager::saveCell(const std::string& cellId, const PM::DataPoints& cell)
{
}

norlab_icp_mapper::PM::DataPoints norlab_icp_mapper::HardDriveCellManager::retrieveCell(const std::string& cellId) const
{
}

void norlab_icp_mapper::HardDriveCellManager::clearAllCells()
{
}
