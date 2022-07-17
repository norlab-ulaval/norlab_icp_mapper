//
// Created by mbo on 19/05/22.
//

#include "ram_cell_manager.h"
#include "RAMCellManager.h"

namespace python
{
	namespace module
	{
		void pybindRAMCellManagerModule(py::module& p_module)
		{
			using RAMCellManager = norlab_icp_mapper::RAMCellManager;
			py::class_<RAMCellManager>(p_module, "RamCellManager")
				.def("getAllCellIds", &RAMCellManager::getAllCellIds)
				.def("saveCell", &RAMCellManager::saveCell, py::arg("cellId"), py::arg("cell"))
				.def("retrieveCell", &RAMCellManager::retrieveCell, py::arg("cellId"))
				.def("clearAllCells", &RAMCellManager::clearAllCells)
				.def(py::init(), "Constructor");
		}
	}
}