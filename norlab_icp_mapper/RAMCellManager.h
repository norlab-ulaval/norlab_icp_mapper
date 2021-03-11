#include "CellManager.h"
#include <unordered_map>

namespace norlab_icp_mapper
{
	class RAMCellManager : public CellManager
	{
	private:
		std::unordered_map<std::string, PM::DataPoints> cells;

	public:
		std::vector<std::string> getAllCellIds() const override;
		void saveCell(const std::string& cellId, const PM::DataPoints& cell) override;
		PM::DataPoints retrieveCell(const std::string& cellId) const override;
		void clearAllCells() override;
	};
}
