#ifndef NORLAB_ICP_MAPPER_CELLINFO_H
#define NORLAB_ICP_MAPPER_CELLINFO_H

#include <pointmatcher/PointMatcher.h>

namespace norlab_icp_mapper
{
	struct CellInfo
	{
		int row;
		int column;
		int aisle;
		int depth;

		CellInfo(const int& row, const int& column, const int& aisle, const int& depth);
		CellInfo(const std::string& name);
		std::string getName() const;
		bool operator==(const CellInfo& other) const;
	};
}

namespace std
{
	template<>
	struct hash<norlab_icp_mapper::CellInfo>
	{
		size_t operator()(const norlab_icp_mapper::CellInfo& cellInfo) const;
	};
}

#endif
