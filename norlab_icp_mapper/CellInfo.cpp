#include <CellInfo.h>

norlab_icp_mapper::CellInfo::CellInfo(const int& row, const int& column, const int& aisle, const int& depth) :
		row(row), column(column), aisle(aisle), depth(depth)
{
}

norlab_icp_mapper::CellInfo::CellInfo(const std::string& name)
{
	size_t rowEndPos = name.find("_");
	std::string rowStr = name.substr(0, rowEndPos);
	row = std::stoi(rowStr);
	size_t columnEndPos = name.find("_", rowEndPos + 1);
	std::string columnStr = name.substr(rowEndPos + 1, columnEndPos);
	column = std::stoi(columnStr);
	size_t aisleEndPos = name.find("_", columnEndPos + 1);
	std::string aisleStr = name.substr(columnEndPos + 1, aisleEndPos);
	aisle = std::stoi(aisleStr);
	std::string depthStr = name.substr(aisleEndPos + 1);
	depth = std::stoi(depthStr);
}

std::string norlab_icp_mapper::CellInfo::getName() const
{
	return std::to_string(row) + "_" + std::to_string(column) + "_" + std::to_string(aisle) + "_" + std::to_string(depth);
}

bool norlab_icp_mapper::CellInfo::operator==(const CellInfo& other) const
{
	return getName() == other.getName();
}

size_t std::hash<norlab_icp_mapper::CellInfo>::operator()(const norlab_icp_mapper::CellInfo& cellInfo) const
{
	return std::hash<std::string>()(cellInfo.getName());
}
