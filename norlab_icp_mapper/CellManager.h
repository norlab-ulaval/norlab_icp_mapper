#ifndef CELL_MANAGER_H
#define CELL_MANAGER_H

#include <pointmatcher/PointMatcher.h>

namespace norlab_icp_mapper
{
    class CellManager
    {
    protected:
        typedef PointMatcher<float> PM;

    public:
        virtual ~CellManager() = default;
        virtual std::vector<std::string> getAllCellIds() const = 0;
        virtual void saveCell(const std::string& cellId, const PM::DataPoints& cell) = 0;
        virtual PM::DataPoints retrieveCell(const std::string& cellId) const = 0;
        virtual void clearAllCells() = 0;
    };
}

#endif
